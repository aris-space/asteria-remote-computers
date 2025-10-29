use std::{error::Error, io::Write, time::Duration};
use tokio::time::sleep;
use ur20::ModuleType;
use ur20_modbus::Coupler;

const MODULES: [ModuleType; 9] = [
    ModuleType::UR20_8DI_P_2W,    // Slot 11: 8 inputs (proximity sensors)
    ModuleType::UR20_4AI_TC_DIAG, // Slot 12: 4 thermo couples
    ModuleType::UR20_4AI_UI_16,   // Slot 13: 4 analog inputs (?)
    ModuleType::UR20_8DO_P,       // Slot 14: 8 outputs (valves)
    // Slot 15: power feed (passive)
    ModuleType::UR20_8DO_P,        // Slot 16: 8 outputs (valves)
    ModuleType::UR20_8DO_P,        // Slot 17: 8 outputs (valves)
    ModuleType::UR20_4DO_P_2A,     // Slot 18: 4 2A outputs (valves)
    ModuleType::UR20_PF_O_1DI_SIL, // Slot 19: Safe power feed
    ModuleType::UR20_4DO_P_2A,     // Slot 20: 4 2A outputs (ignitors)
];

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    let addr = "192.168.0.222:502".parse()?;
    let mut coupler = Coupler::connect(addr).await?;
    let id = coupler.id().await?;
    println!("Connected to {id}");

    dbg!(coupler.modules());
    dbg!(coupler.modules() == MODULES);

    coupler.tick().await.expect("Modbus error");

    dbg!(coupler.inputs());
    dbg!(coupler.outputs());

    coupler
        .set_output(
            &ur20::Address {
                module: 3, // Slot 14: first UR20_8DO_P
                channel: 2,
            },
            ur20::ChannelValue::Bit(true),
        )
        .expect("unable to write");

    let mut bit = true;

    loop {
        print!(".");
        let _ = std::io::stdout().flush();
        coupler
            .set_output(
                &ur20::Address {
                    module: 3,  // Slot 14: first UR20_8DO_P
                    channel: 0, // Topmost output
                },
                ur20::ChannelValue::Bit(bit),
            )
            .expect("unable to write");
        coupler
            .set_output(
                &ur20::Address {
                    module: 3,  // Slot 14: first UR20_8DO_P
                    channel: 1, // Second output from top
                },
                ur20::ChannelValue::Bit(!bit),
            )
            .expect("unable to write");
        bit = !bit;
        coupler.tick().await.expect("Modbus error");
        sleep(Duration::from_millis(200)).await;
    }
}
