# Flashing LVDS Streaming Firmware to IWR1443

## Problem
The radar is not transmitting LVDS data because it doesn't have the correct firmware loaded.
The test shows the radar echoes commands but doesn't execute them - this means it's running
default/factory firmware, not the LVDS streaming firmware.

## Solution
Flash the LVDS streaming firmware to the radar's flash memory.

## Required Firmware
Location: `/home/arik/rosattempt2/iwr_raw_rosnode/firmware/xwr14xx_lvds_stream.bin`

## Method 1: Using TI UniFlash (RECOMMENDED)

### Download UniFlash
- Get from: https://www.ti.com/tool/UNIFLASH
- Available for Linux (and Windows)
- Free tool from TI

### Flashing Steps
1. **Set jumpers to Development Mode:**
   - SOP0 = 1 (jumper ON)
   - SOP1 = 1 (jumper ON)  
   - SOP2 = 0 (jumper OFF)
   - Power cycle the radar

2. **Connect hardware:**
   - USB cable from radar to PC
   - Radar should appear as XDS110 debug probe

3. **Run UniFlash:**
   ```bash
   # After installing UniFlash
   cd /path/to/uniflash
   ./uniflash.sh
   ```

4. **In UniFlash GUI:**
   - Select device: IWR1443
   - Select connection: XDS110
   - Browse to binary: `/home/arik/rosattempt2/iwr_raw_rosnode/firmware/xwr14xx_lvds_stream.bin`
   - Flash address: 0x00000000 (default for SPI flash)
   - Click "Load Image"

5. **After flashing completes:**
   - Set jumpers back to Functional Mode:
     * SOP0 = 0 (jumper OFF)
     * SOP1 = 0 (jumper OFF)
     * SOP2 = 0 (jumper OFF)
   - Power cycle the radar

## Method 2: Using Code Composer Studio (CCS)

If you have CCS installed:

1. **Set jumpers to Development Mode** (same as above)

2. **In CCS:**
   - Target Configurations → New Target Configuration
   - Select IWR1443
   - Connection: Texas Instruments XDS110 USB Debug Probe
   - Launch debugger
   - Tools → Flash → Flash Programmer
   - Browse to binary file
   - Flash to address 0x00000000

3. **Set jumpers back to Functional Mode** after flashing

## Method 3: Using mmWave Studio (If you have it)

1. **Set jumpers to Development Mode**
2. **Open mmWave Studio**
3. **Connect to radar**
4. **Flash firmware:**
   - Firmware → Download Firmware
   - Select: `/home/arik/rosattempt2/iwr_raw_rosnode/firmware/xwr14xx_lvds_stream.bin`
   - Flash

5. **Set jumpers back to Functional Mode**

## Verification After Flashing

Run this test to verify firmware is working:

```bash
python3 /home/arik/rosattempt2/check_radar_firmware.py
```

**Expected output after correct firmware:**
- Radar should respond with "Done" to commands
- Version command should show firmware info
- sensorStart/sensorStop should be recognized

**Current output (wrong firmware):**
- Radar echoes commands but doesn't execute
- No "Done" responses
- Commands not recognized

## Why This is Necessary

The IWR1443 can run different applications:
- **Factory firmware**: Basic serial interface, no LVDS streaming
- **LVDS streaming firmware**: Custom firmware that streams ADC data via LVDS to DCA1000

When you used mmWave Studio, it may have loaded firmware temporarily to RAM but didn't
flash it to persistent storage. In Functional Mode (SOP jumpers OFF), the radar boots
from flash, not from what mmWave Studio loaded.

## Alternative: Quick Test with mmWave Studio

If you still have mmWave Studio working:
1. Keep jumpers in Development Mode (SOP0=1, SOP1=1)
2. Load firmware via mmWave Studio
3. Try running the ROS node while mmWave Studio has the firmware loaded

This won't persist across power cycles but will confirm if the firmware is the issue.

## Next Steps After Flashing

Once firmware is flashed and verified:
1. Power on with Functional Mode jumpers (SOP0=0, SOP1=0)
2. Run: `./cleanup_radar.sh`
3. Run: `./run_radar.sh`
4. You should see data flowing!

## Download Links

- **TI UniFlash**: https://www.ti.com/tool/UNIFLASH
- **Code Composer Studio**: https://www.ti.com/tool/CCSTUDIO
- **mmWave SDK**: https://www.ti.com/tool/MMWAVE-SDK

## Troubleshooting

**Q: UniFlash doesn't see my radar**
- Check jumpers are in Development Mode
- Check USB cable connection
- Try different USB port
- On Linux: check permissions for USB device

**Q: Flashing fails**
- Verify you selected correct device (IWR1443)
- Check connection type is XDS110
- Try power cycling radar
- Check flash address is 0x00000000

**Q: After flashing, radar still doesn't work**
- Verify jumpers are back in Functional Mode
- Power cycle the radar completely
- Run firmware verification test
- Check serial output with check_radar_firmware.py
