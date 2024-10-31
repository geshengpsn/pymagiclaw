use std::{thread::sleep, time::Duration};

use tungstenite::{connect, Message};

fn main() {
    let (mut ws, _) = connect("ws://192.168.5.24:6666").unwrap();
    ws.send(Message::Text("Calibration".into())).unwrap();
    sleep(Duration::from_secs_f64(5.));
    for _ in 0..100 {
        sleep(Duration::from_secs_f64(0.033));
        ws.send(Message::Binary((0.5f32).to_le_bytes().to_vec())).unwrap();
    }
    ws.close(None).unwrap();
}