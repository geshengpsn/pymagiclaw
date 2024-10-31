use franka::{exception::FrankaException, Frame, MotionFinished, RealtimeConfig, Robot, Torques};
use nalgebra::{
    Isometry3, Matrix3, Matrix4, Matrix6, Matrix6x1, Rotation3, SMatrix, UnitQuaternion, Vector3,
};
use numpy::{ndarray::Array2, ToPyArray};
use pyo3::prelude::*;
use std::{
    io,
    sync::{
        mpsc::{channel, Sender},
        Arc, Mutex, RwLock, TryLockError,
    },
    thread::spawn,
};

/// A Python-friendly wrapper for controlling a Franka robot.
#[pyclass]
pub struct Franka {
    inner: Arc<Mutex<FrankaInner>>,
    session: Option<ControlSession>,
}

struct FrankaInner {
    robot: Robot,
}

struct ControlSession {
    control_msg_tx: Sender<ControlMsg>,
    state: Arc<RwLock<[f64; 16]>>,
}

enum ControlMsg {
    RelativeCartesian(Isometry3<f64>),
    AbsoluteCartesian(Isometry3<f64>),
    Stop,
}

impl Drop for Franka {
    fn drop(&mut self) {
        self.stop().unwrap();
    }
}

#[pymethods]
impl Franka {
    /// Create a new Franka robot instance.
    ///
    /// Args:
    ///     address (str): The IP address of the robot.
    ///     realtime (bool): Whether to use realtime mode.
    ///
    /// Returns:
    ///     Franka: A new Franka robot instance.
    #[new]
    pub fn new(address: String, realtime: bool) -> PyResult<Self> {
        let mut robot = Robot::new(
            address.as_str(),
            if realtime {
                None
            } else {
                Some(RealtimeConfig::Ignore)
            },
            None,
        )
        .map_err(io::Error::other)?;
        robot
            .set_collision_behavior(
                [100.; 7], [100.; 7], [100.; 7], [100.; 7], [100.; 6], [100.; 6], [100.; 6],
                [100.; 6],
            )
            .unwrap();
        robot
            .set_joint_impedance([3000., 3000., 3000., 2500., 2500., 2000., 2000.])
            .unwrap();
        robot
            .set_cartesian_impedance([3000., 3000., 3000., 300., 300., 300.])
            .unwrap();
        let inner = Arc::new(Mutex::new(FrankaInner { robot }));
        Ok(Franka {
            inner,
            session: None,
        })
    }

    /// Start the control session for the robot.
    ///
    /// Args:
    ///     translational_stiffness (float): The translational stiffness parameter for the controller.
    ///     rotational_stiffness (float): The rotational stiffness parameter for the controller.
    ///
    /// Returns:
    ///     None
    pub fn start_control(
        &mut self,
        translational_stiffness: f64,
        rotational_stiffness: f64,
    ) -> PyResult<()> {
        if self.session.is_some() {
            return Err(std::io::Error::other("robot in use, start new control failed").into());
        }
        let (control_msg_tx, control_msg_rx) = channel::<ControlMsg>();

        let state_rwlock = Arc::new(RwLock::new([0.; 16]));
        let state_rwlock_clone = state_rwlock.clone();

        let inner_copy = self.inner.clone();
        let _handle = spawn(move || {
            let mut inner_guard = inner_copy
                .try_lock()
                .map_err(|e| match e {
                    TryLockError::Poisoned(_) => {
                        std::io::Error::other("internal error, should not happend")
                    }
                    TryLockError::WouldBlock => {
                        std::io::Error::other("robot in use, start new control failed")
                    }
                })
                .unwrap();
            let model = inner_guard.robot.load_model(true).unwrap();
            // let mut robot_ee_pose_d = Isometry3::identity();
            let state = inner_guard.robot.read_once().unwrap();
            let mut robot_ee_pose_d = Isometry3::from_parts(
                Vector3::new(state.O_T_EE[12], state.O_T_EE[13], state.O_T_EE[14]).into(),
                Rotation3::<f64>::from_matrix(
                    &Matrix4::from_column_slice(&state.O_T_EE)
                        .remove_column(3)
                        .remove_row(3),
                )
                .into(),
            );
            let (stiffness_matrix, damping_matrix) =
                stiffness_damping(translational_stiffness, rotational_stiffness);
            let e = inner_guard.robot.control_torques(
                |state, _time| {
                    {
                        let mut state_writer = state_rwlock.write().unwrap();
                        *state_writer = state.O_T_EE;
                    }
                    match control_msg_rx.try_recv() {
                        Ok(ControlMsg::RelativeCartesian(delta)) => {
                            robot_ee_pose_d *= delta;
                        }
                        Ok(ControlMsg::AbsoluteCartesian(delta)) => {
                            robot_ee_pose_d = delta;
                        }
                        Err(std::sync::mpsc::TryRecvError::Disconnected) | Ok(ControlMsg::Stop) => {
                            return Torques::new([0., 0., 0., 0., 0., 0., 0.]).motion_finished();
                        }
                        _ => {}
                    };

                    let robot_ee_pose = Isometry3::from_parts(
                        Vector3::new(state.O_T_EE[12], state.O_T_EE[13], state.O_T_EE[14]).into(),
                        Rotation3::<f64>::from_matrix(
                            &Matrix4::from_column_slice(&state.O_T_EE)
                                .remove_column(3)
                                .remove_row(3),
                        )
                        .into(),
                    );

                    let position = robot_ee_pose.translation.vector;
                    let mut orientation = *robot_ee_pose.rotation.quaternion();

                    let position_d = robot_ee_pose_d.translation.vector;
                    let orientation_d: UnitQuaternion<f64> = robot_ee_pose_d.rotation;

                    let coriolis: SMatrix<f64, 7, 1> = model.coriolis_from_state(state).into();
                    let jacobian_array = model.zero_jacobian_from_state(&Frame::EndEffector, state);
                    let jacobian = SMatrix::<f64, 6, 7>::from_column_slice(&jacobian_array);
                    // let _q = Vector7::from_column_slice(&state.q);
                    let dq = SMatrix::<f64, 7, 1>::from_column_slice(&state.dq);

                    let mut error: Matrix6x1<f64> = Matrix6x1::<f64>::zeros();
                    {
                        let mut error_head = error.fixed_view_mut::<3, 1>(0, 0);
                        error_head.set_column(0, &(position - position_d));
                    }

                    if orientation_d.coords.dot(&orientation.coords) < 0. {
                        orientation.coords = -orientation.coords;
                    }
                    let orientation = UnitQuaternion::new_normalize(orientation);
                    let error_quaternion: UnitQuaternion<f64> =
                        orientation.inverse() * orientation_d;
                    {
                        let mut error_tail = error.fixed_view_mut::<3, 1>(3, 0);
                        error_tail.copy_from(
                            &-(robot_ee_pose.rotation.to_rotation_matrix()
                                * Vector3::new(
                                    error_quaternion.i,
                                    error_quaternion.j,
                                    error_quaternion.k,
                                )),
                        );
                    }
                    let tau_task = jacobian.transpose()
                        * (-stiffness_matrix * error - damping_matrix * (jacobian * dq));
                    let tau_d = tau_task + coriolis;
                    Torques::new([
                        tau_d[0], tau_d[1], tau_d[2], tau_d[3], tau_d[4], tau_d[5], tau_d[6],
                    ])
                },
                None,
                None,
            );
            if let Err(FrankaException::ControlException { log: _, error }) = e {
                println!("{error}")
            }
        });

        self.session = Some(ControlSession {
            control_msg_tx,
            state: state_rwlock_clone,
        });

        Ok(())
    }

    /// Move the robot relative to its current position in Cartesian space.
    ///
    /// Args:
    ///     delta_cartesian (numpy.ndarray): A 4x4 transformation matrix representing the relative movement.
    ///
    /// Returns:
    ///     None
    pub fn move_relative_cartesian(
        &mut self,
        delta_cartesian: numpy::PyReadonlyArray2<f64>,
    ) -> PyResult<()> {
        match self.session.as_ref() {
            Some(session) => {
                let m = Matrix4::from_row_slice(delta_cartesian.as_slice()?);
                let delta = Isometry3::from_parts(
                    Vector3::new(m[(0, 3)], m[(1, 3)], m[(2, 3)]).into(),
                    Rotation3::<f64>::from_matrix(&m.remove_column(3).remove_row(3)).into(),
                );

                session
                    .control_msg_tx
                    .send(ControlMsg::RelativeCartesian(delta))
                    .map_err(std::io::Error::other)?;
                Ok(())
            }
            None => Err(std::io::Error::other(
                "no control session active, please call start_control first",
            )
            .into()),
        }
    }

    /// Move the robot to an absolute position in Cartesian space.
    ///
    /// Args:
    ///     cartesian (numpy.ndarray): A 4x4 transformation matrix representing the absolute position.
    ///
    /// Returns:
    ///     None
    pub fn move_absolute_cartesian(
        &mut self,
        cartesian: numpy::PyReadonlyArray2<f64>,
    ) -> PyResult<()> {
        match self.session.as_ref() {
            Some(session) => {
                let m = Matrix4::from_row_slice(cartesian.as_slice()?);
                let delta = Isometry3::from_parts(
                    Vector3::new(m[(0, 3)], m[(1, 3)], m[(2, 3)]).into(),
                    Rotation3::<f64>::from_matrix(&m.remove_column(3).remove_row(3)).into(),
                );

                session
                    .control_msg_tx
                    .send(ControlMsg::AbsoluteCartesian(delta))
                    .map_err(std::io::Error::other)?;
                Ok(())
            }
            None => Err(std::io::Error::other(
                "no control session active, please call start_control first",
            )
            .into()),
        }
    }

    /// Read the current state of the robot.
    ///
    /// Returns:
    ///     numpy.ndarray: A 4x4 transformation matrix representing the current end-effector pose.
    pub fn read_state<'py>(
        &mut self,
        py: Python<'py>,
    ) -> PyResult<Bound<'py, numpy::PyArray2<f64>>> {
        match self.session.as_ref() {
            Some(session) => {
                let state = session.state.read().unwrap();
                let array = Array2::from_shape_vec((4, 4), state.to_vec()).unwrap();
                let res = array.t().to_pyarray_bound(py);
                Ok(res)
            }
            None => {
                let mut inner_lock = self.inner.lock().unwrap();
                let state = inner_lock.robot.read_once().unwrap();
                let array = Array2::from_shape_vec((4, 4), state.O_T_EE.to_vec()).unwrap();
                let res = array.t().to_pyarray_bound(py);
                Ok(res)
            }
        }
    }

    /// Stop the current control session.
    ///
    /// Returns:
    ///     None
    pub fn stop(&mut self) -> PyResult<()> {
        match self.session.take() {
            Some(session) => {
                session
                    .control_msg_tx
                    .send(ControlMsg::Stop)
                    .map_err(std::io::Error::other)?;
                Ok(())
            }
            None => Err(std::io::Error::other(
                "no control session active, please call start_control first",
            )
            .into()),
        }
    }
}

fn stiffness_damping(
    translational_stiffness: f64,
    rotational_stiffness: f64,
) -> (Matrix6<f64>, Matrix6<f64>) {
    let mut stiffness = SMatrix::<f64, 6, 6>::zeros();
    let mut damping = SMatrix::<f64, 6, 6>::zeros();
    {
        let mut top_left_corner = stiffness.fixed_view_mut::<3, 3>(0, 0);
        top_left_corner.copy_from(&(Matrix3::identity() * translational_stiffness));
        let mut top_left_corner = damping.fixed_view_mut::<3, 3>(0, 0);
        top_left_corner.copy_from(&(2. * f64::sqrt(translational_stiffness) * Matrix3::identity()));
    }
    {
        let mut bottom_right_corner = stiffness.fixed_view_mut::<3, 3>(3, 3);
        bottom_right_corner.copy_from(&(Matrix3::identity() * rotational_stiffness));
        let mut bottom_right_corner = damping.fixed_view_mut::<3, 3>(3, 3);
        bottom_right_corner
            .copy_from(&(2. * f64::sqrt(rotational_stiffness) * Matrix3::identity()));
    }
    (stiffness, damping)
}

pub(crate) fn add_franka_submodule(parent_module: &Bound<'_, PyModule>) -> PyResult<()> {
    let child_module = PyModule::new_bound(parent_module.py(), "franka")?;
    // child_module.add_function(wrap_pyfunction!(func, &child_module)?)?;
    child_module.add_class::<Franka>()?;
    // child_module.add_class::<ConnectConfig>()?;
    parent_module.add_submodule(&child_module)
}
