extern crate RobotCode2024;
use RobotCode2024::{telemetry::{self, TELEMETRY}, constants::TELEMETRY_PORT};

#[tokio::main]
async fn main() {
    let router = telemetry::server()
        .with_state(TELEMETRY.clone());
    let listener = tokio::net::TcpListener::bind(format!("0.0.0.0:{}", TELEMETRY_PORT)).await.unwrap();
    axum::serve(listener, router).await.unwrap();
}
