use std::collections::HashMap;

use crate::blocking::client::Client;
use crate::error::Result;

/// Synchronous blocking control handle for articulated or scripted environment scenery actors.
#[derive(Clone)]
pub struct EnvActor {
    inner: crate::async_api::EnvActor,
    client: Client,
}

impl EnvActor {
    /// Creates a new blocking EnvActor handle.
    pub fn new(
        client: Client,
        actor_name: impl Into<String>,
        world_parent_topic: impl Into<String>,
    ) -> Self {
        let inner =
            crate::async_api::EnvActor::new(client.inner().clone(), actor_name, world_parent_topic);
        Self { inner, client }
    }

    /// Creates a blocking EnvActor handle wrapping an existing async handle.
    pub fn from_inner(inner: crate::async_api::EnvActor, client: Client) -> Self {
        Self { inner, client }
    }

    /// Returns the actor's unique name.
    pub fn name(&self) -> &str {
        self.inner.name()
    }

    /// Assigns a trajectory asset to this environment actor.
    #[allow(clippy::too_many_arguments)]
    pub fn set_trajectory(
        &self,
        traj_name: &str,
        to_loop: bool,
        time_offset: f32,
        x_offset: f32,
        y_offset: f32,
        z_offset: f32,
        roll_offset: f32,
        pitch_offset: f32,
        yaw_offset: f32,
    ) -> Result<bool> {
        self.client.runtime().block_on(self.inner.set_trajectory(
            traj_name,
            to_loop,
            time_offset,
            x_offset,
            y_offset,
            z_offset,
            roll_offset,
            pitch_offset,
            yaw_offset,
        ))
    }

    /// Sets the rotation angle in degrees for an articulated link.
    pub fn set_link_rotation_angle(&self, link_name: &str, angle_deg: f32) -> Result<bool> {
        self.client
            .runtime()
            .block_on(self.inner.set_link_rotation_angle(link_name, angle_deg))
    }

    /// Sets the rotation rate in degrees per second for an articulated link.
    pub fn set_link_rotation_rate(
        &self,
        link_name: &str,
        rotation_deg_per_sec: f32,
    ) -> Result<bool> {
        self.client
            .runtime()
            .block_on(self.inner.set_link_rotation_rate(link_name, rotation_deg_per_sec))
    }

    /// Sets rotation angles for multiple articulated links sequentially.
    pub fn set_link_rotation_angles(&self, angles: &HashMap<String, f32>) -> Result<()> {
        self.client
            .runtime()
            .block_on(self.inner.set_link_rotation_angles(angles))
    }

    /// Sets rotation rates for multiple articulated links sequentially.
    pub fn set_link_rotation_rates(&self, rates: &HashMap<String, f32>) -> Result<()> {
        self.client
            .runtime()
            .block_on(self.inner.set_link_rotation_rates(rates))
    }
}
