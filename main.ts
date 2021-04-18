/**

*
* This code is released under the [MIT License](http://opensource.org/licenses/MIT).
* Please review the LICENSE.md file included with this example. If you have any questions 
* or concerns with licensing, please contact techsupport@sparkfun.com.
* Distributed as-is; no warranty is given.
*/


const enum DistanceUnit {
  //% block="cm"
  CM = 58, // Duration of echo round-trip in Microseconds (uS) for two centimeters, 343 m/s at sea level and 20°C
  //% block="inch"
  INCH = 148, // Duration of echo round-trip in Microseconds (uS) for two inches, 343 m/s at sea level and 20°C
}

/**
 * Functions to operate the CCS811
 */


//% color=#33acff icon="\uf06c"
namespace cyberGarden {

  //Start of CCS811 air quality sensor 
	//Keep track of CCS811 Start 
	let appStarted = false;

  //CCS811 Addresses
  const ccsAddr = 0x5B
  const ccsStatus = 0x00
  const ccsMeas = 0x01
  const ccsAlg = 0x02
  const ccsRaw = 0x03
  const ccsEnv = 0x05
  const ccsNtc = 0x06
  const ccsThres = 0x10
  const ccsBase = 0x11
  const ccsHi = 0x20
  const ccsHv = 0x21
  const ccsBoot = 0x23
  const ccsAppv = 0x24
  const ccsErr = 0xE0
  const ccsApps = 0xF4
  const ccsReset = 0xFF

	/**
     *  Easy test for ensuring I2C read is working
     */

    //% weight=1 blockId="hardwareID" block="HWID"
    export function hardwareID(): number {
        let hardwareId = readCCSReg(0x20, NumberFormat.UInt8LE)
        return hardwareId
    }


    /**
     * Gets the CO2 data from the algorithm register
     * of the CCS811 Air Quality Sensor
     */

    //% weight=100 blockId="readCo2" block="Read eCO2"
    export function readCo2(): number {
        //read Algorithm Results register

        let algRes = readCCSReg(ccsAlg, NumberFormat.UInt16BE)
        return algRes
    }

    /**
     * Gets the TVOC data from the algorithm register
     * of the CCS811 Air Quality Sensor
     */

    //% weight=90 blockId="readTvoc" block="Read TVOCs"
    export function readTvoc(): number {
        //read Algorithm Results register
        let algRes = readCCSReg(ccsAlg, NumberFormat.Int32BE)
        let Tvoc = (algRes & 0x0000FFFF)
        return Tvoc
    }

    //% weight=2 blockId="readStatus" block="Device Status"
    export function readStatus(): number {
        //Return status of Device
        let status = readCCSReg(ccsStatus, NumberFormat.UInt8LE)
        return status
    }

    /**
     * Read the device error code if there are
     * any problems with device
     */

    //% weight=3 blockId="readError" block="Device Error"
    export function readError(): number {
        //Return Error of Device
        let error = readCCSReg(ccsErr, NumberFormat.Int8LE)
        return error
    }


	/**
     * Writes a value to a register on the CCS811 Air Quality Sensor
     */
    function writeCCSReg(reg: number, val: number): void {
        let test = reg << 8 | val
        pins.i2cWriteNumber(ccsAddr, reg << 8 | val, NumberFormat.Int16BE)
    }

	/**
     * Reads a value from a register on the CCS811 Air Quality Sensor
     */
    function readCCSReg(reg: number, format: NumberFormat) {
        pins.i2cWriteNumber(ccsAddr, reg, NumberFormat.UInt8LE, false)
        let val = pins.i2cReadNumber(ccsAddr, format, false)
        return val
    }


	/**
     * Gets the CCS811 into app mode, and sets the measure register
     * to pull data into Algorithm register every second. 
     */

    //% weight=100 blockId="AppStart" block="CCS811 Start"
    export function appStart(): void {
		if (appStarted) return;
		
        pins.i2cWriteNumber(ccsAddr, ccsApps, NumberFormat.Int8LE)
        writeCCSReg(ccsMeas, 0x10)
		
		//init once 
		appStarted = true;
    }
  
  
  
  //Start of BME280 Environemntal sensor 
    let environmentMonitorStarted = false;
    // Keep Track of environment monitoring variables
   // BME280 Addresses
    const bmeAddr = 0x77
    const ctrlHum = 0xF2
    const ctrlMeas = 0xF4
    const config = 0xF5
    const pressMSB = 0xF7
    const pressLSB = 0xF8
    const pressXlsb = 0xF9
    const tempMSB = 0xFA
    const tempLSB = 0xFB
    const tempXlsb = 0xFC
    const humMSB = 0xFD
    const humLSB = 0xFE

    // Stores compensation values for Temperature (must be read from BME280 NVM)
    let digT1Val = 0
    let digT2Val = 0
    let digT3Val = 0

    // Stores compensation values for humidity (must be read from BME280 NVM)
    let digH1Val = 0
    let digH2Val = 0
    let digH3Val = 0
    let digH4Val = 0
    let digH5Val = 0
    let digH6Val = 0

    // Buffer to hold pressure compensation values to pass to the C++ compensation function
    let digPBuf: Buffer

    // BME Compensation Parameter Addresses for Temperature
    const digT1 = 0x88
    const digT2 = 0x8A
    const digT3 = 0x8C

    // BME Compensation Parameter Addresses for Pressure
    const digP1 = 0x8E
    const digP2 = 0x90
    const digP3 = 0x92
    const digP4 = 0x94
    const digP5 = 0x96
    const digP6 = 0x98
    const digP7 = 0x9A
    const digP8 = 0x9C
    const digP9 = 0x9E

    // BME Compensation Parameter Addresses for Humidity
    const digH1 = 0xA1
    const digH2 = 0xE1
    const digH3 = 0xE3
    const e5Reg = 0xE5
    const e4Reg = 0xE4
    const e6Reg = 0xE6
    const digH6 = 0xE7
    
      /***************************************************************************************
     * Functions for interfacing with the BME280
     ***************************************************************************************/

    /**
     * Writes a value to a register on the BME280
     */
    function WriteBMEReg(reg: number, val: number): void {
        pins.i2cWriteNumber(bmeAddr, reg << 8 | val, NumberFormat.Int16BE)
    }

    /**
     * Reads a value from a register on the BME280
     */
    function readBMEReg(reg: number, format: NumberFormat) {
        pins.i2cWriteNumber(bmeAddr, reg, NumberFormat.UInt8LE, false)
        let val = pins.i2cReadNumber(bmeAddr, format, false)
        return val
    }

    /**
     * Reads the temp from the BME sensor and uses compensation for calculator temperature.
     * Returns 4 digit number. Value should be devided by 100 to get DegC
     */
    //% weight=43 blockGap=8 blockId="temperature" block="temperature(C)"
    export function temperature(): number {
        // Read the temperature registers
        let tempRegM = readBMEReg(tempMSB, NumberFormat.UInt16BE)
        let tempRegL = readBMEReg(tempXlsb, NumberFormat.UInt8LE)

        // Use compensation formula and return result
        return compensateTemp((tempRegM << 4) | (tempRegL >> 4))
    }

    /**
     * Reads the humidity from the BME sensor and uses compensation for calculating humidity.
     * returns a 5 digit number. Value should be divided by 1024 to get % relative humidity. 
     */
    //% weight=41 blockGap=8 blockId="humidity" block="humidity"
    export function humidity(): number {
        // Read the pressure registers
        let humReg = readBMEReg(humMSB, NumberFormat.UInt16BE)

        // Compensate and return humidity
        return compensateHumidity(humReg)
    }

    /**
     * Reads the pressure from the BME sensor and uses compensation for calculating pressure.
     * Returns an 8 digit number. Value should be divided by 25600 to get hPa. 
     */
    //% weight=42 blockGap=8 blockId="pressure" block="pressure"
    export function pressure(): number {
        // Read the temperature registers
        let pressRegM = readBMEReg(pressMSB, NumberFormat.UInt16BE)
        let pressRegL = readBMEReg(pressXlsb, NumberFormat.UInt8LE)

        // Compensate and return pressure
        return compensatePressure((pressRegM << 4) | (pressRegL >> 4), tFine, digPBuf)
    }

    /**
     * Sets up BME for in Environment Monitoring Mode.
     */
    //% weight=44 blockGap=8 blockId="setupBME280" block="start environment monitoring"
    export function startEnvironmentMonitoring(): void {
        if (environmentMonitorStarted) return;

        // The 0xE5 register is 8 bits where 4 bits go to one value and 4 bits go to another
        let e5Val = 0

        // Instantiate buffer that holds the pressure compensation values
        digPBuf = pins.createBuffer(18)

        // Set up the BME in environment monitoring mode
        WriteBMEReg(ctrlHum, 0x01)
        WriteBMEReg(ctrlMeas, 0x27)
        WriteBMEReg(config, 0)

        // Read the temperature registers to do a calculation and set tFine
        let tempRegM = readBMEReg(tempMSB, NumberFormat.UInt16BE)
        let tempRegL = readBMEReg(tempXlsb, NumberFormat.UInt8LE)

        // Get the NVM digital compensations numbers from the device for temp
        digT1Val = readBMEReg(digT1, NumberFormat.UInt16LE)
        digT2Val = readBMEReg(digT2, NumberFormat.Int16LE)
        digT3Val = readBMEReg(digT3, NumberFormat.Int16LE)

        // Get the NVM digital compensation number from the device for pressure and pack into
        // a buffer to pass to the C++ implementation of the compensation formula
        digPBuf.setNumber(NumberFormat.UInt16LE, 0, readBMEReg(digP1, NumberFormat.UInt16LE))
        digPBuf.setNumber(NumberFormat.Int16LE, 2, readBMEReg(digP2, NumberFormat.Int16LE))
        digPBuf.setNumber(NumberFormat.Int16LE, 4, readBMEReg(digP3, NumberFormat.Int16LE))
        digPBuf.setNumber(NumberFormat.Int16LE, 6, readBMEReg(digP4, NumberFormat.Int16LE))
        digPBuf.setNumber(NumberFormat.Int16LE, 8, readBMEReg(digP5, NumberFormat.Int16LE))
        digPBuf.setNumber(NumberFormat.Int16LE, 10, readBMEReg(digP6, NumberFormat.Int16LE))
        digPBuf.setNumber(NumberFormat.Int16LE, 12, readBMEReg(digP7, NumberFormat.Int16LE))
        digPBuf.setNumber(NumberFormat.Int16LE, 14, readBMEReg(digP8, NumberFormat.Int16LE))
        digPBuf.setNumber(NumberFormat.Int16LE, 16, readBMEReg(digP9, NumberFormat.Int16LE))

        // Get the NVM digital compensation number from device for humidity
        e5Val = readBMEReg(e5Reg, NumberFormat.Int8LE)
        digH1Val = readBMEReg(digH1, NumberFormat.UInt8LE)
        digH2Val = readBMEReg(digH2, NumberFormat.Int16LE)
        digH3Val = readBMEReg(digH3, NumberFormat.UInt8LE)
        digH4Val = (readBMEReg(e4Reg, NumberFormat.Int8LE) << 4) | (e5Val & 0xf)
        digH5Val = (readBMEReg(e6Reg, NumberFormat.Int8LE) << 4) | (e5Val >> 4)
        digH6Val = readBMEReg(digH6, NumberFormat.Int8LE)

        // Compensate the temperature to calcule the tFine variable for use in other
        // measurements
        let temp = compensateTemp((tempRegM << 4) | (tempRegL >> 4))

        environmentMonitorStarted = true;
    }

    // Global variable used in all calculations for the BME280
    let tFine = 0

    /**
     * Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
     * tFine carries fine temperature as global value
     */
    function compensateTemp(tempRegVal: number): number {
        let firstConv: number = (((tempRegVal >> 3) - (digT1Val << 1)) * digT2Val) >> 11
        let secConv: number = (((((tempRegVal >> 4) - digT1Val) * ((tempRegVal >> 4) - (digT1Val))) >> 12) * (digT3Val)) >> 14
        tFine = firstConv + secConv
        return (tFine * 5 + 128) >> 8
    }

    /**
     * Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22 integer and 10 fractional bits).
     * Output value of “47445” represents 47445/1024 = 46.333 %RH
     */
    function compensateHumidity(humRegValue: number): number {
        let hum: number = (tFine - 76800)
        hum = (((((humRegValue << 14) - (digH4Val << 20) - (digH5Val * hum)) + 16384) >> 15) * (((((((hum * digH6Val) >> 10) * (((hum * digH3Val) >> 11) + 32768)) >> 10) + 2097152) * digH2Val + 8192) >> 14))
        hum = hum - (((((hum >> 15) * (hum >> 15)) >> 7) * digH1Val) >> 4)
        hum = (hum < 0 ? 0 : hum)
        hum = (hum > 419430400 ? 419430400 : hum)
        return (hum >> 12)
    }

    /**
     * Function used for simulator, actual implementation is in weatherbit.cpp
     */
    //%
    function compensatePressure(pressRegVal: number, tFine: number, compensation: Buffer) {
        let digP1: number = (compensation[0] << 8) | compensation[1];
        let digP2: number = (compensation[2] << 8) | compensation[3];
        let digP3: number = (compensation[4] << 8) | compensation[5];
        let digP4: number = (compensation[6] << 8) | compensation[7];
        let digP5: number = (compensation[8] << 8) | compensation[9];
        let digP6: number = (compensation[10] << 8) | compensation[11];
        let digP7: number = (compensation[12] << 8) | compensation[13];
        let digP8: number = (compensation[14] << 8) | compensation[15];
        let digP9: number = (compensation[16] << 8) | compensation[17];

        let firstConv: number = tFine - 128000;
        let secondConv: number = firstConv * firstConv * digP6;
        secondConv += firstConv * digP5 << 17;
        secondConv += digP4 << 35;
        firstConv = ((firstConv * firstConv * digP3) >> 8) + ((firstConv * digP2) << 12);
        firstConv = ((1 << 47) + firstConv) * (digP1 >> 33);
        if (firstConv == 0) {
            return 69; //avoid exception caused by divide by 0
        }
        let pressureReturn = 1048576 - pressRegVal;
        pressureReturn = (((pressureReturn << 31) - secondConv) * 3125) / firstConv;
        firstConv = (digP9 * (pressureReturn << 13) * (pressureReturn << 13)) >> 25;
        secondConv = (digP8 * pressureReturn) >> 19;
        pressureReturn = ((pressureReturn + firstConv + secondConv) >> 8) + (digP7 << 4);
        return pressureReturn;
    }

    /**
   * Reads the pressure from the BME sensor and uses compensation for calculating pressure.
   * Returns altitude in meters based on pressure at sea level. (absolute altitude)
   */
    //% weight=40 blockGap=28 blockId="altitude" block="altitude(M)"
    export function altitude(): number {
        startEnvironmentMonitoring();

        let pressRegM = readBMEReg(pressMSB, NumberFormat.UInt16BE)
        let pressRegL = readBMEReg(pressXlsb, NumberFormat.UInt8LE)
        return calcAltitude((pressRegM << 4) | (pressRegL >> 4), tFine, digPBuf)
    }

    /** 
     * Function used for simulator, actual implementation is in weatherbit.cpp
     */
    //%
    function calcAltitude(pressRegVal: number, tFine: number, compensation: Buffer): number {
        let returnValue = compensatePressure(pressRegVal, tFine, compensation);
        returnValue /= 25600.0;
        returnValue /= 1013.25;
        returnValue = returnValue ** 0.1903;
        returnValue = 1 - returnValue;
        returnValue *= 44330;
        return returnValue;
    }
  
  
   /**
     * Reads the Moisture Level from the Soil Moisture Sensor.
	 * Returns a value between 0 and 1023. 0 being dry and 1023 being wet.     
     */
    //% weight=11 blockGap=8 blockId="soilMoisture" block="soil moisture"
    export function soilMoisture(): number {
        let soilMoisture = 0
        soilMoisture = pins.analogReadPin(AnalogPin.P1)
        basic.pause(1000)
        return soilMoisture
    }

   /***************************************************************************************
    * Functions for interfacing with the HC-SR04 ultrasonic distance sensor 
    ***************************************************************************************/
  
  const MICROBIT_MAKERBIT_ULTRASONIC_OBJECT_DETECTED_ID = 798;
  const MAX_ULTRASONIC_TRAVEL_TIME = 300 * DistanceUnit.CM;
  const ULTRASONIC_MEASUREMENTS = 3;

  interface UltrasonicRoundTrip {
    ts: number;
    rtt: number;
  }

  interface UltrasonicDevice {
    trig: DigitalPin | undefined;
    roundTrips: UltrasonicRoundTrip[];
    medianRoundTrip: number;
    travelTimeObservers: number[];
  }

  let ultrasonicState: UltrasonicDevice;

  /**
   * Configures the ultrasonic distance sensor and measures continuously in the background.
   * @param trig pin connected to trig, eg: DigitalPin.P5
   * @param echo pin connected to echo, eg: DigitalPin.P8
   */
  //% subcategory="Ultrasonic"
  //% blockId="makerbit_ultrasonic_connect"
  //% block="connect ultrasonic distance sensor | with Trig at %trig | and Echo at %echo"
  //% trig.fieldEditor="gridpicker"
  //% trig.fieldOptions.columns=4
  //% trig.fieldOptions.tooltips="false"
  //% echo.fieldEditor="gridpicker"
  //% echo.fieldOptions.columns=4
  //% echo.fieldOptions.tooltips="false"
  //% weight=80
  export function connectUltrasonicDistanceSensor(
    trig: DigitalPin,
    echo: DigitalPin
  ): void {
    if (ultrasonicState && ultrasonicState.trig) {
      return;
    }

    if (!ultrasonicState) {
      ultrasonicState = {
        trig: trig,
        roundTrips: [{ ts: 0, rtt: MAX_ULTRASONIC_TRAVEL_TIME }],
        medianRoundTrip: MAX_ULTRASONIC_TRAVEL_TIME,
        travelTimeObservers: [],
      };
    } else {
      ultrasonicState.trig = trig;
    }

    pins.onPulsed(echo, PulseValue.High, () => {
      if (
        pins.pulseDuration() < MAX_ULTRASONIC_TRAVEL_TIME &&
        ultrasonicState.roundTrips.length <= ULTRASONIC_MEASUREMENTS
      ) {
        ultrasonicState.roundTrips.push({
          ts: input.runningTime(),
          rtt: pins.pulseDuration(),
        });
      }
    });

    control.inBackground(measureInBackground);
  }

  /**
   * Do something when an object is detected the first time within a specified range.
   * @param distance distance to object, eg: 20
   * @param unit unit of distance, eg: DistanceUnit.CM
   * @param handler body code to run when the event is raised
   */
  //% subcategory="Ultrasonic"
  //% blockId=makerbit_ultrasonic_on_object_detected
  //% block="on object detected once within | %distance | %unit"
  //% weight=69
  export function onUltrasonicObjectDetected(
    distance: number,
    unit: DistanceUnit,
    handler: () => void
  ) {
    if (distance <= 0) {
      return;
    }

    if (!ultrasonicState) {
      ultrasonicState = {
        trig: undefined,
        roundTrips: [{ ts: 0, rtt: MAX_ULTRASONIC_TRAVEL_TIME }],
        medianRoundTrip: MAX_ULTRASONIC_TRAVEL_TIME,
        travelTimeObservers: [],
      };
    }

    const travelTimeThreshold = Math.imul(distance, unit);

    ultrasonicState.travelTimeObservers.push(travelTimeThreshold);

    control.onEvent(
      MICROBIT_MAKERBIT_ULTRASONIC_OBJECT_DETECTED_ID,
      travelTimeThreshold,
      () => {
        handler();
      }
    );
  }

  /**
   * Returns the distance to an object in a range from 1 to 300 centimeters or up to 118 inch.
   * The maximum value is returned to indicate when no object was detected.
   * -1 is returned when the device is not connected.
   * @param unit unit of distance, eg: DistanceUnit.CM
   */
  //% subcategory="Ultrasonic"
  //% blockId="makerbit_ultrasonic_distance"
  //% block="ultrasonic distance in %unit"
  //% weight=60
  export function getUltrasonicDistance(unit: DistanceUnit): number {
    if (!ultrasonicState) {
      return -1;
    }
    basic.pause(0); // yield to allow background processing when called in a tight loop
    return Math.idiv(ultrasonicState.medianRoundTrip, unit);
  }

  /**
   * Returns `true` if an object is within the specified distance. `false` otherwise.
   *
   * @param distance distance to object, eg: 20
   * @param unit unit of distance, eg: DistanceUnit.CM
   */
  //% subcategory="Ultrasonic"
  //% blockId="makerbit_ultrasonic_less_than"
  //% block="ultrasonic distance is less than | %distance | %unit"
  //% weight=50
  export function isUltrasonicDistanceLessThan(
    distance: number,
    unit: DistanceUnit
  ): boolean {
    if (!ultrasonicState) {
      return false;
    }
    basic.pause(0); // yield to allow background processing when called in a tight loop
    return Math.idiv(ultrasonicState.medianRoundTrip, unit) < distance;
  }

  function triggerPulse() {
    // Reset trigger pin
    pins.setPull(ultrasonicState.trig, PinPullMode.PullNone);
    pins.digitalWritePin(ultrasonicState.trig, 0);
    control.waitMicros(2);

    // Trigger pulse
    pins.digitalWritePin(ultrasonicState.trig, 1);
    control.waitMicros(10);
    pins.digitalWritePin(ultrasonicState.trig, 0);
  }

  function getMedianRRT(roundTrips: UltrasonicRoundTrip[]) {
    const roundTripTimes = roundTrips.map((urt) => urt.rtt);
    return median(roundTripTimes);
  }

  // Returns median value of non-empty input
  function median(values: number[]) {
    values.sort((a, b) => {
      return a - b;
    });
    return values[(values.length - 1) >> 1];
  }

  function measureInBackground() {
    const trips = ultrasonicState.roundTrips;
    const TIME_BETWEEN_PULSE_MS = 145;

    while (true) {
      const now = input.runningTime();

      if (trips[trips.length - 1].ts < now - TIME_BETWEEN_PULSE_MS - 10) {
        ultrasonicState.roundTrips.push({
          ts: now,
          rtt: MAX_ULTRASONIC_TRAVEL_TIME,
        });
      }

      while (trips.length > ULTRASONIC_MEASUREMENTS) {
        trips.shift();
      }

      ultrasonicState.medianRoundTrip = getMedianRRT(
        ultrasonicState.roundTrips
      );

      for (let i = 0; i < ultrasonicState.travelTimeObservers.length; i++) {
        const threshold = ultrasonicState.travelTimeObservers[i];
        if (threshold > 0 && ultrasonicState.medianRoundTrip <= threshold) {
          control.raiseEvent(
            MICROBIT_MAKERBIT_ULTRASONIC_OBJECT_DETECTED_ID,
            threshold
          );
          // use negative sign to indicate that we notified the event
          ultrasonicState.travelTimeObservers[i] = -threshold;
        } else if (
          threshold < 0 &&
          ultrasonicState.medianRoundTrip > -threshold
        ) {
          // object is outside the detection threshold -> re-activate observer
          ultrasonicState.travelTimeObservers[i] = -threshold;
        }
      }

      triggerPulse();
      basic.pause(TIME_BETWEEN_PULSE_MS);
    }
  }
  
}
  
  
  
