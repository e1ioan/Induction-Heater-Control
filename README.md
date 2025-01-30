NOTE: This is a work in progress

## Required Components

Below is a list of components needed for this project:

- **ATTINY85 Development Board**  
- **I2C LCD 1602 Module (2x16)**  
- **MP1584EN DC to DC Buck Converter** (3A, 7V~28V to 5V)  
- **DC 5V-36V 15A (MAX 30A) 400W Dual High Power MOSFET Trigger Switch Drive Module**  
- **ACS712 Current Sensor (20A module)**  
- **MLX90614 Non-Contact Infrared Sensor**  
- **Momentary Push Button**  
- **Glass 14mm Female to 18mm Male Short Expander Adapter Connector**  
- **DC 5V-12V ZVS DIY Low Voltage Induction Heating Module**  
- **LEDs**  

Total price as of January 2025, in Europe, is under 50€ (~$52)
Make sure to gather all these components before proceeding with the build.

[![The controller, test video](https://img.youtube.com/vi/F0CzHAuRhXs/0.jpg)](https://youtube.com/shorts/F0CzHAuRhXs)

First of all I recommend replacing the n-channel mosfets that came on the ZVS induction module with something better. 

## Recommended N-Channel MOSFETs for ZVS Induction Coil (12V, 120W)

For a **12V, 120W ZVS induction coil**, you need N-channel MOSFETs that can handle at least **20-30A** with a low Rds(on) and fast switching times.

### Key Considerations:
1. **Voltage Rating (Vds):**  
   - At least **2-3x your supply voltage**, so a **30V to 60V** MOSFET is ideal.
   
2. **Current Rating (Id):**  
   - **Minimum 20-30A**, but ideally **higher (~50A)** for reliability.
   
3. **Rds(on) (On-Resistance):**  
   - The lower, the better (< **10mΩ** preferred).
   
4. **Gate Drive Requirements:**  
   - Logic-level MOSFETs (Vgs(th) ≤ 2V) if using a low-voltage gate driver.

### Good MOSFET Choices:

| MOSFET          | Vds (Max) | Id (Max) | Rds(on)  | Notes                                |
|----------------|----------|---------|---------|-------------------------------------|
| **IRLZ44N**     | 55V      | 47A     | 16mΩ    | Cheap and easy to find              |
| **IRLB3034PBF** | 40V      | 195A    | 1.4mΩ   | Super low Rds(on), great efficiency |
| **IRF3205**     | 55V      | 110A    | 8mΩ     | Common choice, but not logic-level  |
| **IPB034N06N**  | 60V      | 50A     | 3.4mΩ   | Better efficiency, lower heat       |
| **STP55NF06**   | 60V      | 55A     | 14mΩ    | Decent mid-range option             |

### Best Pick for Efficiency:
- **IRLB3034PBF** (lowest Rds(on), highest current handling, but more expensive).
- **IPB034N06N** (good mix of low resistance and current capability).

### Best Pick for Budget Build:
- **IRLZ44N** or **IRF3205** (widely available, still decent performance).
