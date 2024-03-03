use std::{sync::Arc, collections::HashMap};
use axum::{Router, extract::{State, Path}, routing::get};
use num_traits::{ToPrimitive, FromPrimitive};
use serde::Serialize;
use tokio::sync::RwLock;

use crate::auto::Auto;

pub type TelemetryStore = Arc<RwLock<Telemetry>>;

#[derive(Serialize)]
pub enum Data {
    Number(f64),
    Bool(bool),
    Text(String),
}

#[derive(Default)]
pub struct Telemetry {
    pub auto: Auto,
    pub data: HashMap<String, Data>
}

pub fn server() -> Router<TelemetryStore> {
    let router = Router::new()
        .route("/get_auto", get(get_auto))
        .route("/set_auto/:id", get(set_auto)); // words have no meaning :)

    router
}

async fn get_auto(
    State(state): State<TelemetryStore>) -> String {
    state.read().await.auto.to_owned().to_usize().unwrap().to_string()
}

async fn set_auto(
    State(state): State<TelemetryStore>,
    Path(auto): Path<usize>,
    ) -> &'static str {
    if let Some(auto) = Auto::from_usize(auto) {
        state.write().await.auto = auto;

        "Success"
    } else {
        "Failure"
    }

}
