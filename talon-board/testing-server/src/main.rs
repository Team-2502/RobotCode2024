extern crate RobotCode2024;
use RobotCode2024::{
    constants::TELEMETRY_PORT,
    telemetry::{self, put_number, Pose, TELEMETRY},
};

#[tokio::main]
async fn main() {
    TELEMETRY
        .write()
        .await
        .data
        .insert("hertz".to_owned(), telemetry::Data::Number(8.));
    put_number("rio load", 0.322024).await;
    TELEMETRY
        .write()
        .await
        .data
        .insert("flywheel rpm".to_owned(), telemetry::Data::Number(5000.));
    TELEMETRY
        .write()
        .await
        .data
        .insert("intake status".to_owned(), telemetry::Data::Bool(true));
    TELEMETRY
        .write()
        .await
        .data
        .insert("text".to_owned(), telemetry::Data::Text("lorem".to_owned()));
    TELEMETRY.write().await.data.insert(
        "Auto Setpoint".to_owned(),
        telemetry::Data::Pose(Pose {
            x: 1.0,
            y: 1.0,
            theta: 3.14,
        }),
    );

    let router = telemetry::server().with_state(TELEMETRY.clone());
    let listener = tokio::net::TcpListener::bind(format!("0.0.0.0:{}", TELEMETRY_PORT))
        .await
        .unwrap();
    axum::serve(listener, router).await.unwrap();
}
