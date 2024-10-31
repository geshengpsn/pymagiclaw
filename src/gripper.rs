use std::net::TcpStream;

use pyo3::prelude::*;
use tungstenite::{connect, stream::MaybeTlsStream, WebSocket};

#[pyclass]
struct Gripper {
    ws: WebSocket<MaybeTlsStream<TcpStream>>,
}


impl Drop for Gripper {
    fn drop(&mut self) {
        print!("closed");
        self.ws.close(None).unwrap();
    }
}

#[pymethods]
impl Gripper {
    #[new]
    fn new(ip: &str) -> Gripper {
        let req = "ws://".to_string() + ip + ":6666";
        let (ws, _) = connect(&req).unwrap();
        Gripper { ws }
    }

    fn calibration(&mut self) {
        self.ws
            .send(tungstenite::Message::Text("Calibration".into()))
            .unwrap();
    }

    fn pos(&mut self, pos: f32) {
        self.ws
            .send(tungstenite::Message::Binary(pos.to_le_bytes().to_vec()))
            .unwrap();
    }
}

pub(crate) fn add_gripper_submodule(parent_module: &Bound<'_, PyModule>) -> PyResult<()> {
    let child_module = PyModule::new_bound(parent_module.py(), "gripper")?;
    // child_module.add_function(wrap_pyfunction!(func, &child_module)?)?;
    child_module.add_class::<Gripper>()?;
    // child_module.add_class::<ConnectConfig>()?;
    parent_module.add_submodule(&child_module)
}
