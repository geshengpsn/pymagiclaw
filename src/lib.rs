use pyo3::prelude::*;
mod franka;
/// Formats the sum of two numbers as string.
#[pyfunction]
fn sum_as_string(a: usize, b: usize) -> PyResult<String> {
    Ok((a + b).to_string())
}

/// A Python module implemented in Rust.
#[pymodule]
fn pymagiclaw(m: &Bound<'_, PyModule>) -> PyResult<()> {
    crate::franka::add_franka_submodule(m)?;
    m.add_function(wrap_pyfunction!(sum_as_string, m)?)?;
    Ok(())
}
