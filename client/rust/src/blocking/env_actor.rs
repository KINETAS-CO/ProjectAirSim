use std::collections::HashMap;
use tracing::info;

use crate::blocking::client::Client;
use crate::error::Result;
use crate::protocol::params::{
    SetLinkRotationAngleParams, SetLinkRotationRateParams, SetTrajectoryParams,
};

/// Synchronous blocking control handle for articulated or scripted environment scenery actors.
#[derive(Clone)]
pub struct EnvActor {
    client: Client,
    actor_name: String,
    world_parent_topic: String,
}

impl EnvActor {
    /// Creates a new blocking EnvActor handle.
    pub fn new(
        client: Client,
        actor_name: impl Into<String>,
        world_parent_topic: impl Into<String>,
    ) -> Self {
        Self {
            client,
            actor_name: actor_name.into(),
            world_parent_topic: world_parent_topic.into(),
        }
    }

    /// Returns the actor's unique name.
    pub fn name(&self) -> &str {
        &self.actor_name
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
        info!(
            "Setting trajectory '{}' on env actor '{}'",
            traj_name, self.actor_name
        );
        let path = format!("{}/SetEnvActorTrajectory", self.world_parent_topic);
        self.client.request(
            &path,
            &SetTrajectoryParams {
                env_actor_name: &self.actor_name,
                traj_name,
                time_offset,
                x_offset,
                y_offset,
                z_offset,
                roll_offset,
                pitch_offset,
                yaw_offset,
                to_loop,
            },
        )
    }

    /// Sets the rotation angle in degrees for an articulated link.
    pub fn set_link_rotation_angle(&self, link_name: &str, angle_deg: f32) -> Result<bool> {
        let path = format!("{}/SetEnvActorLinkRotAngle", self.world_parent_topic);
        self.client.request(
            &path,
            &SetLinkRotationAngleParams {
                env_actor_name: &self.actor_name,
                link_name,
                angle_deg,
            },
        )
    }

    /// Sets the rotation rate in degrees per second for an articulated link.
    pub fn set_link_rotation_rate(
        &self,
        link_name: &str,
        rotation_deg_per_sec: f32,
    ) -> Result<bool> {
        let path = format!("{}/SetEnvActorLinkRotRate", self.world_parent_topic);
        self.client.request(
            &path,
            &SetLinkRotationRateParams {
                env_actor_name: &self.actor_name,
                link_name,
                rotation_deg_per_sec,
            },
        )
    }

    /// Sets rotation angles for multiple articulated links sequentially.
    pub fn set_link_rotation_angles(&self, angles: &HashMap<String, f32>) -> Result<()> {
        for (link, &angle) in angles {
            self.set_link_rotation_angle(link, angle)?;
        }
        Ok(())
    }

    /// Sets rotation rates for multiple articulated links sequentially.
    pub fn set_link_rotation_rates(&self, rates: &HashMap<String, f32>) -> Result<()> {
        for (link, &rate) in rates {
            self.set_link_rotation_rate(link, rate)?;
        }
        Ok(())
    }
}
