use hermes_can::messages;
use hermes_can::messages::board_status::ValveState;
use lib_mission::actuator_control::Energizable;
use lib_mission::reactive::{IntoWatched as _, Watched, WatchedExt};
use tokio::task::AbortHandle;
use tracing::Instrument as _;

use crate::output::PSSValves;

/// A macro to generate the `fn_name` function, spawning tasks for sending system-specific duplicates of messages.
///
/// The reason for this is that the variants are different structs and different IDs, to not collide.
/// Otherwise the `xxxFC` and `xxxRFS` message variants in [`messages::Message`] are identical, which allows this code reuse using macros.
macro_rules! impl_spawn_drivers {
    (
        $fn_name:ident,
        $doc:literal,
        $prz_vnt_vlv_name:ident,
        $fss_prz_vlv_name:ident,
        $fss_vnt_vlv_name:ident,
        $oss_prz_vlv_name:ident,
        $oss_vnt_vlv_name:ident,
        $fss_mnl_vlv_name:ident,
        $oss_mnl_vlv_name:ident,
    ) => {
        #[doc = $doc]
        pub fn $fn_name(
            &self,
            tx: &std::sync::mpsc::Sender<messages::Message>,
        ) -> [AbortHandle; 7] {
            [
                // Pressurization Vent Valve (`PRZ_VNT_VLV`):
                spawn_output_driver(
                    self.pressurization_vent_valve.watched().map(|target| {
                        messages::Message::$prz_vnt_vlv_name(
                            messages::event_messages::$prz_vnt_vlv_name {
                                prz_vnt_vlv: valve_state_from_energizable(&target),
                            },
                        )
                    }),
                    self.is_active.watched(),
                    tx.clone(),
                    tracing::debug_span!("drivers::pressurization_vent_valve"),
                ),
                // Fuel Pressurization Valve (DPR) (`FSS_PRZ_VLV`):
                spawn_output_driver(
                    self.fuel_pressurization_valve.watched().map(|target| {
                        messages::Message::$fss_prz_vlv_name(
                            messages::event_messages::$fss_prz_vlv_name {
                                fss_dpr: target.into(),
                            },
                        )
                    }),
                    self.is_active.watched(),
                    tx.clone(),
                    tracing::debug_span!("drivers::fuel_pressurization_valve"),
                ),
                // Fuel Vent Valve (`FSS_VNT_VLV`):
                spawn_output_driver(
                    self.fuel_vent_valve.watched().map(|target| {
                        messages::Message::$fss_vnt_vlv_name(
                            messages::event_messages::$fss_vnt_vlv_name {
                                fss_vnt_vlv: valve_state_from_energizable(&target),
                            },
                        )
                    }),
                    self.is_active.watched(),
                    tx.clone(),
                    tracing::debug_span!("drivers::fuel_vent_valve"),
                ),
                // Oxidizer Pressurization Valve (DPR) (`OSS_PRZ_VLV`):
                spawn_output_driver(
                    self.oxidizer_pressurization_valve.watched().map(|target| {
                        messages::Message::$oss_prz_vlv_name(
                            messages::event_messages::$oss_prz_vlv_name {
                                oss_dpr: target.into(),
                            },
                        )
                    }),
                    self.is_active.watched(),
                    tx.clone(),
                    tracing::debug_span!("drivers::oxidizer_pressurization_valve"),
                ),
                // Oxidizer Vent Valve (`OSS_VNT_VLV`):
                spawn_output_driver(
                    self.oxidizer_vent_valve.watched().map(|target| {
                        messages::Message::$oss_vnt_vlv_name(
                            messages::event_messages::$oss_vnt_vlv_name {
                                oss_vnt_vlv: valve_state_from_energizable(&target),
                            },
                        )
                    }),
                    self.is_active.watched(),
                    tx.clone(),
                    tracing::debug_span!("drivers::oxidizer_vent_valve"),
                ),
                // Fuel Main Valve (`FSS_MNL_VLV`):
                spawn_output_driver(
                    self.fuel_main_valve.watched().map(|target| {
                        messages::Message::$fss_mnl_vlv_name(
                            messages::event_messages::$fss_mnl_vlv_name {
                                fss_mnl_vlv: valve_state_from_energizable(&target),
                            },
                        )
                    }),
                    self.is_active.watched(),
                    tx.clone(),
                    tracing::debug_span!("drivers::fuel_main_valve"),
                ),
                // Oxidizer Main Valve (`OSS_MNL_VLV`):
                spawn_output_driver(
                    self.oxidizer_main_valve.watched().map(|target| {
                        messages::Message::$oss_mnl_vlv_name(
                            messages::event_messages::$oss_mnl_vlv_name {
                                oss_mnl_vlv: valve_state_from_energizable(&target),
                            },
                        )
                    }),
                    self.is_active.watched(),
                    tx.clone(),
                    tracing::debug_span!("drivers::oxidizer_main_valve"),
                ),
            ]
        }
    };
}

impl PSSValves {
    impl_spawn_drivers! {
        spawn_output_drivers_fc,
        "Spawns the CAN drivers sending control messages from the FC",
        PressurizationVentControlFC,
        FuelDprConfigFC,
        FuelVentControlFC,
        OxidizerDprConfigFC,
        OxidizerVentControlFC,
        FuelMainControlFC,
        OxidizerMainControlFC,
    }

    impl_spawn_drivers! {
        spawn_output_drivers_rfs,
        "Spawns the CAN drivers sending control messages from the RFS",
        PressurizationVentControlRFS,
        FuelDprConfigRFS,
        FuelVentControlRFS,
        OxidizerDprConfigRFS,
        OxidizerVentControlRFS,
        FuelMainControlRFS,
        OxidizerMainControlRFS,
    }
}

/// Gets the CAN active/inactive state based on an `Energizable`
fn valve_state_from_energizable(valve: &impl Energizable) -> ValveState {
    if valve.is_active() {
        ValveState::Active
    } else {
        ValveState::Inactive
    }
}

/// Spawns a tokio task with continuously reads the `watched` and sends it to the transmit channel `tx`
/// while `enabled` is `true`. Otherwise it discards all changes.
///
/// Runs in a tracing span `span`.
/// This returns the abort handle with should be used to cancel the task, otherwise the task will give a warning when `tx` closes.
fn spawn_output_driver<M>(
    mut watched: impl Watched<M> + Send + 'static,
    enabled: impl Watched<bool> + Send + 'static,
    tx: std::sync::mpsc::Sender<M>,
    span: tracing::Span,
) -> tokio::task::AbortHandle
where
    M: Clone + Send + 'static,
{
    tokio::spawn(
        async move {
            loop {
                let msg = watched.next().await;
                if enabled.current() {
                    if tx.send(msg).is_err() {
                        tracing::warn!("CAN channel was closed. Shutting down!");
                        break;
                    }
                } else {
                    tracing::trace!("skipping disabled output");
                }
            }
        }
        .instrument(span),
    )
    .abort_handle()
}
