import requests
import time
from bs4 import BeautifulSoup

class FlightAxis:
    """
    Python version of Ardupilot's FlightAxis implementation, available here:
    - https://github.com/ArduPilot/ardupilot/blob/master/libraries/SITL/SIM_FlightAxis.h
    - https://github.com/ArduPilot/ardupilot/blob/master/libraries/SITL/SIM_FlightAxis.cpp

    Example of FlightAxis response is here: http://uav.tridgell.net/RealFlight/data-exchange.txt

    More details about FlightAxis interface:
    - https://www.knifeedge.com/forums/index.php?threads/flightaxis-link-q-a.32854/
    - https://www.knifeedge.com/forums/index.php?threads/flightaxis-link-network-access-to-realflight-for-developers.32809/#post-282152


    # Performance
    A typical performance measured on a reasonable desktop PC is:
    avg dt = 0.0042965165828869264[s], avg Freq = 232.746686928432

    """
    REALFLIGHT_URL = "http://127.0.0.1:18083"
    RC_CHANNLES = 12

    def getKeytable(self) -> dict:
        keytable = {
            "m-airspeed-MPS": "self.m_airspeed_MPS",
            "m-altitudeASL-MTR": "self.m_altitudeASL_MTR",
            "m-altitudeAGL-MTR": "self.m_altitudeAGL_MTR",
            "m-groundspeed-MPS": "self.m_groundspeed_MPS",
            "m-pitchRate-DEGpSEC": "self.m_pitchRate_DEGpSEC",
            "m-rollRate-DEGpSEC": "self.m_rollRate_DEGpSEC",
            "m-yawRate-DEGpSEC": "self.m_yawRate_DEGpSEC",
            "m-azimuth-DEG": "self.m_azimuth_DEG",
            "m-inclination-DEG": "self.m_inclination_DEG",
            "m-roll-DEG": "self.m_roll_DEG",
            "m-aircraftPositionX-MTR": "self.m_aircraftPositionX_MTR",
            "m-aircraftPositionY-MTR": "self.m_aircraftPositionY_MTR",
            "m-velocityWorldU-MPS": "self.m_velocityWorldU_MPS",
            "m-velocityWorldV-MPS": "self.m_velocityWorldV_MPS",
            "m-velocityWorldW-MPS": "self.m_velocityWorldW_MPS",
            "m-velocityBodyU-MPS": "self.m_velocityBodyU_MPS",
            "m-velocityBodyV-MPS": "self.m_velocityBodyV_MPS",
            "m-velocityBodyW-MPS": "self.m_velocityBodyW_MPS",
            "m-accelerationWorldAX-MPS2": "self.m_accelerationWorldAX_MPS2",
            "m-accelerationWorldAY-MPS2": "self.m_accelerationWorldAY_MPS2",
            "m-accelerationWorldAZ-MPS2": "self.m_accelerationWorldAZ_MPS2",
            "m-accelerationBodyAX-MPS2": "self.m_accelerationBodyAX_MPS2",
            "m-accelerationBodyAY-MPS2": "self.m_accelerationBodyAY_MPS2",
            "m-accelerationBodyAZ-MPS2": "self.m_accelerationBodyAZ_MPS2",
            "m-windX-MPS": "self.m_windX_MPS",
            "m-windY-MPS": "self.m_windY_MPS",
            "m-windZ-MPS": "self.m_windZ_MPS",
            "m-propRPM": "self.m_propRPM",
            "m-heliMainRotorRPM": "self.m_heliMainRotorRPM",
            "m-batteryVoltage-VOLTS": "self.m_batteryVoltage_VOLTS",
            "m-batteryCurrentDraw-AMPS": "self.m_batteryCurrentDraw_AMPS",
            "m-batteryRemainingCapacity-MAH": "self.m_batteryRemainingCapacity_MAH",
            "m-fuelRemaining-OZ": "self.m_fuelRemaining_OZ",
            "m-isLocked": "self.m_isLocked",
            "m-hasLostComponents": "self.m_hasLostComponents",
            "m-anEngineIsRunning": "self.m_anEngineIsRunning",
            "m-isTouchingGround": "self.m_isTouchingGround",
            # TODO: needs better string handling here
            # "m-currentAircraftStatus": "self.m_currentAircraftStatus",
            "m-currentPhysicsTime-SEC": "self.m_currentPhysicsTime_SEC",
            "m-currentPhysicsSpeedMultiplier": "self.m_currentPhysicsSpeedMultiplier",
            "m-orientationQuaternion-X": "self.m_orientationQuaternion_X",
            "m-orientationQuaternion-Y": "self.m_orientationQuaternion_Y",
            "m-orientationQuaternion-Z": "self.m_orientationQuaternion_Z",
            "m-orientationQuaternion-W": "self.m_orientationQuaternion_W",
        }
        return keytable

    def __init__(self):
        self.last_frame_time = time.time()
        self.avg_dt = 0.0

        self.rcin = [0.0 for _ in range(FlightAxis.RC_CHANNLES)]
        self.m_airspeed_MPS = 0.0
        self.m_altitudeASL_MTR = 0.0
        self.m_altitudeAGL_MTR = 0.0
        self.m_groundspeed_MPS = 0.0
        self.m_pitchRate_DEGpSEC = 0.0
        self.m_rollRate_DEGpSEC = 0.0
        self.m_yawRate_DEGpSEC = 0.0
        self.m_azimuth_DEG = 0.0
        self.m_inclination_DEG = 0.0
        self.m_roll_DEG = 0.0
        self.m_aircraftPositionX_MTR = 0.0
        self.m_aircraftPositionY_MTR = 0.0
        self.m_velocityWorldU_MPS = 0.0
        self.m_velocityWorldV_MPS = 0.0
        self.m_velocityWorldW_MPS = 0.0
        self.m_velocityBodyU_MPS = 0.0
        self.m_velocityBodyV_MPS = 0.0
        self.m_velocityBodyW_MPS = 0.0
        self.m_accelerationWorldAX_MPS2 = 0.0
        self.m_accelerationWorldAY_MPS2 = 0.0
        self.m_accelerationWorldAZ_MPS2 = 0.0
        self.m_accelerationBodyAX_MPS2 = 0.0
        self.m_accelerationBodyAY_MPS2 = 0.0
        self.m_accelerationBodyAZ_MPS2 = 0.0
        self.m_windX_MPS = 0.0
        self.m_windY_MPS = 0.0
        self.m_windZ_MPS = 0.0
        self.m_propRPM = 0.0
        self.m_heliMainRotorRPM = 0.0
        self.m_batteryVoltage_VOLTS = 0.0
        self.m_batteryCurrentDraw_AMPS = 0.0
        self.m_batteryRemainingCapacity_MAH = 0.0
        self.m_fuelRemaining_OZ = 0.0
        self.m_isLocked = 0.0
        self.m_hasLostComponents = 0.0
        self.m_anEngineIsRunning = 0.0
        self.m_isTouchingGround = 0.0
        self.m_currentAircraftStatus = "None"
        self.m_currentPhysicsTime_SEC = 0.0
        self.m_currentPhysicsSpeedMultiplier = 0.0
        self.m_orientationQuaternion_X = 0.0
        self.m_orientationQuaternion_Y = 0.0
        self.m_orientationQuaternion_Z = 0.0
        self.m_orientationQuaternion_W = 0.0
        self.m_flightAxisControllerIsActive = False
        self.m_resetButtonHasBeenPressed = False

    def enableRC(self, doPrint=False) -> bool:
        """
        Set Spektrum as the RC input
        Return True if the response was OK
        """
        headers = {'content-type': "text/xml;charset='UTF-8'",
                'soapaction': 'RestoreOriginalControllerDevice',
                'Connection': 'Keep-Alive'}

        body = "<?xml version='1.0' encoding='UTF-8'?>\
        <soap:Envelope xmlns:soap='http://schemas.xmlsoap.org/soap/envelope/' xmlns:xsd='http://www.w3.org/2001/XMLSchema' xmlns:xsi='http://www.w3.org/2001/XMLSchema-instance'>\
        <soap:Body>\
        <RestoreOriginalControllerDevice><a>1</a><b>2</b></RestoreOriginalControllerDevice>\
        </soap:Body>\
        </soap:Envelope>"

        response = requests.post(FlightAxis.REALFLIGHT_URL,data=body,headers=headers)
        if doPrint:
            print(response.content)
        return response.ok
        
    def disableRC(self, doPrint=False) -> bool:
        """
        Disable Spektrum as the RC input, and use FlightAxis instead
        Return True if the response was OK
        """
        headers = {'content-type': "text/xml;charset='UTF-8'",
                'soapaction': 'InjectUAVControllerInterface'}

        body = "<?xml version='1.0' encoding='UTF-8'?>\
        <soap:Envelope xmlns:soap='http://schemas.xmlsoap.org/soap/envelope/' xmlns:xsd='http://www.w3.org/2001/XMLSchema' xmlns:xsi='http://www.w3.org/2001/XMLSchema-instance'>\
        <soap:Body>\
        <InjectUAVControllerInterface><a>1</a><b>2</b></InjectUAVControllerInterface>\
        </soap:Body>\
        </soap:Envelope>"

        response = requests.post(FlightAxis.REALFLIGHT_URL, data=body,headers=headers)
        if doPrint:
            print(response.content)
        return response.ok

    def getStates(self, doPrint=False) -> bool:
        """
        Set the control inputs, and get the states
        Return True if the response was OK
        """
        headers = {'content-type': "text/xml;charset='UTF-8'",
                'soapaction': 'ExchangeData'}

        body = f"<?xml version='1.0' encoding='UTF-8'?><soap:Envelope xmlns:soap='http://schemas.xmlsoap.org/soap/envelope/' xmlns:xsd='http://www.w3.org/2001/XMLSchema' xmlns:xsi='http://www.w3.org/2001/XMLSchema-instance'>\
        <soap:Body>\
        <ExchangeData>\
        <pControlInputs>\
        <m-selectedChannels>4095</m-selectedChannels>\
        <m-channelValues-0to1>\
        <item>{self.rcin[0]}</item>\
        <item>{self.rcin[1]}</item>\
        <item>{self.rcin[2]}</item>\
        <item>{self.rcin[3]}</item>\
        <item>{self.rcin[4]}</item>\
        <item>{self.rcin[5]}</item>\
        <item>{self.rcin[6]}</item>\
        <item>{self.rcin[7]}</item>\
        <item>{self.rcin[8]}</item>\
        <item>{self.rcin[9]}</item>\
        <item>{self.rcin[10]}</item>\
        <item>{self.rcin[11]}</item>\
        </m-channelValues-0to1>\
        </pControlInputs>\
        </ExchangeData>\
        </soap:Body>\
        </soap:Envelope>"

        response = requests.post(FlightAxis.REALFLIGHT_URL,data=body,headers=headers)

        if doPrint:
            print(response.content)
        
        if response.ok:
            now = time.time()
            dt = now - self.last_frame_time
            self.avg_dt = (self.avg_dt + dt)/2.0
            self.last_frame_time = now
            self.parseResponse(response.content)
            return True
        # Catch all returns false
        return False

    def parseResponse(self, xml):
        """
        Update internal states from FlightAxis response
        """
        parsed = BeautifulSoup(xml, 'xml')
        body = parsed.find('m-aircraftState')
        assert(body)

        keytable = self.getKeytable()
        for k,v in keytable.items():
            tag = body.find(k)
            tag_val = tag.string
            if tag_val == "false" or tag_val == "true":
                tag_val = tag_val.capitalize()
            cmd = f"{v} = {tag_val}"
            exec(cmd)

def test_parse1():
    res = b'<?xml version="1.0" encoding="UTF-8"?>\n<SOAP-ENV:Envelope xmlns:SOAP-ENV="http://schemas.xmlsoap.org/soap/envelope/" xmlns:SOAP-ENC="http://schemas.xmlsoap.org/soap/encoding/" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:xsd="http://www.w3.org/2001/XMLSchema"><SOAP-ENV:Body><ReturnData><m-previousInputsState><m-selectedChannels>-1</m-selectedChannels><m-channelValues-0to1 xsi:type="SOAP-ENC:Array" SOAP-ENC:arrayType="xsd:double[12]"><item>1</item><item>0</item><item>0.51503002643585205</item><item>1</item><item>0</item><item>0</item><item>0</item><item>0</item><item>0</item><item>0</item><item>0</item><item>0</item></m-channelValues-0to1></m-previousInputsState><m-aircraftState><m-currentPhysicsTime-SEC>9.4901527954498306</m-currentPhysicsTime-SEC><m-currentPhysicsSpeedMultiplier>1</m-currentPhysicsSpeedMultiplier><m-airspeed-MPS>4.2668099647568329</m-airspeed-MPS><m-altitudeASL-MTR>0.23033138335698844</m-altitudeASL-MTR><m-altitudeAGL-MTR>0.23033138335698844</m-altitudeAGL-MTR><m-groundspeed-MPS>4.2655322162874292</m-groundspeed-MPS><m-pitchRate-DEGpSEC>46.914449474279536</m-pitchRate-DEGpSEC><m-rollRate-DEGpSEC>-64.392902600666275</m-rollRate-DEGpSEC><m-yawRate-DEGpSEC>-95.093221536022611</m-yawRate-DEGpSEC><m-azimuth-DEG>156.77792358398437</m-azimuth-DEG><m-inclination-DEG>-13.11492919921875</m-inclination-DEG><m-roll-DEG>6.6998014450073242</m-roll-DEG><m-orientationQuaternion-X>0.079808555543422699</m-orientationQuaternion-X><m-orientationQuaternion-Y>0.099987812340259552</m-orientationQuaternion-Y><m-orientationQuaternion-Z>-0.97012227773666382</m-orientationQuaternion-Z><m-orientationQuaternion-W>-0.20614470541477203</m-orientationQuaternion-W><m-aircraftPositionX-MTR>18.125473022460937</m-aircraftPositionX-MTR><m-aircraftPositionY-MTR>13.188897132873535</m-aircraftPositionY-MTR><m-velocityWorldU-MPS>-4.0696659088134766</m-velocityWorldU-MPS><m-velocityWorldV-MPS>1.2777262926101685</m-velocityWorldV-MPS><m-velocityWorldW-MPS>0.10441353917121887</m-velocityWorldW-MPS><m-velocityBodyU-MPS>3.1754355430603027</m-velocityBodyU-MPS><m-velocityBodyV-MPS>-2.8336892127990723</m-velocityBodyV-MPS><m-velocityBodyW-MPS>-0.30408719182014465</m-velocityBodyW-MPS><m-accelerationWorldAX-MPS2>-4.4059576988220215</m-accelerationWorldAX-MPS2><m-accelerationWorldAY-MPS2>-6.3290410041809082</m-accelerationWorldAY-MPS2><m-accelerationWorldAZ-MPS2>8.5558643341064453</m-accelerationWorldAZ-MPS2><m-accelerationBodyAX-MPS2>1.2007522583007813</m-accelerationBodyAX-MPS2><m-accelerationBodyAY-MPS2>8.1494748592376709</m-accelerationBodyAY-MPS2><m-accelerationBodyAZ-MPS2>-8.7651233673095703</m-accelerationBodyAZ-MPS2><m-windX-MPS>0</m-windX-MPS><m-windY-MPS>0</m-windY-MPS><m-windZ-MPS>0</m-windZ-MPS><m-propRPM>10893.9326171875</m-propRPM><m-heliMainRotorRPM>-1</m-heliMainRotorRPM><m-batteryVoltage-VOLTS>-1</m-batteryVoltage-VOLTS><m-batteryCurrentDraw-AMPS>-1</m-batteryCurrentDraw-AMPS><m-batteryRemainingCapacity-MAH>-1</m-batteryRemainingCapacity-MAH><m-fuelRemaining-OZ>11.978337287902832</m-fuelRemaining-OZ><m-isLocked>false</m-isLocked><m-hasLostComponents>false</m-hasLostComponents><m-anEngineIsRunning>true</m-anEngineIsRunning><m-isTouchingGround>true</m-isTouchingGround><m-flightAxisControllerIsActive>true</m-flightAxisControllerIsActive><m-currentAircraftStatus>CAS-FLYING</m-currentAircraftStatus></m-aircraftState><m-notifications><m-resetButtonHasBeenPressed>false</m-resetButtonHasBeenPressed></m-notifications></ReturnData></SOAP-ENV:Body></SOAP-ENV:Envelope>'
    fa = FlightAxis()
    fa.parseResponse(res)

def test_parse2():
    res = b'<?xml version="1.0" encoding="UTF-8"?>\n<SOAP-ENV:Envelope xmlns:SOAP-ENV="http://schemas.xmlsoap.org/soap/envelope/" xmlns:SOAP-ENC="http://schemas.xmlsoap.org/soap/encoding/" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:xsd="http://www.w3.org/2001/XMLSchema"><SOAP-ENV:Body><ReturnData><m-previousInputsState><m-selectedChannels>-1</m-selectedChannels><m-channelValues-0to1 xsi:type="SOAP-ENC:Array" SOAP-ENC:arrayType="xsd:double[12]"><item>1</item><item>0</item><item>0.51503002643585205</item><item>1</item><item>0</item><item>0</item><item>0</item><item>0</item><item>0</item><item>0</item><item>0</item><item>0</item></m-channelValues-0to1></m-previousInputsState><m-aircraftState><m-currentPhysicsTime-SEC>9.5060089953476563</m-currentPhysicsTime-SEC><m-currentPhysicsSpeedMultiplier>1</m-currentPhysicsSpeedMultiplier><m-airspeed-MPS>3.973699447829139</m-airspeed-MPS><m-altitudeASL-MTR>0.23263369561650649</m-altitudeASL-MTR><m-altitudeAGL-MTR>0.23263369561650649</m-altitudeAGL-MTR><m-groundspeed-MPS>3.9671952662134293</m-groundspeed-MPS><m-pitchRate-DEGpSEC>65.486709524862817</m-pitchRate-DEGpSEC><m-rollRate-DEGpSEC>-113.29177463687665</m-rollRate-DEGpSEC><m-yawRate-DEGpSEC>-115.36654693618766</m-yawRate-DEGpSEC><m-azimuth-DEG>154.92184448242187</m-azimuth-DEG><m-inclination-DEG>-12.265393257141113</m-inclination-DEG><m-roll-DEG>4.7752151489257812</m-roll-DEG><m-orientationQuaternion-X>0.063606671988964081</m-orientationQuaternion-X><m-orientationQuaternion-Y>0.095200046896934509</m-orientationQuaternion-Y><m-orientationQuaternion-Z>-0.96875286102294922</m-orientationQuaternion-Z><m-orientationQuaternion-W>-0.22001981735229492</m-orientationQuaternion-W><m-aircraftPositionX-MTR>18.143898010253906</m-aircraftPositionX-MTR><m-aircraftPositionY-MTR>13.126790046691895</m-aircraftPositionY-MTR><m-velocityWorldU-MPS>-3.8283603191375732</m-velocityWorldU-MPS><m-velocityWorldV-MPS>1.0403343439102173</m-velocityWorldV-MPS><m-velocityWorldW-MPS>-0.22726421058177948</m-velocityWorldW-MPS><m-velocityBodyU-MPS>2.9091477394104004</m-velocityBodyU-MPS><m-velocityBodyV-MPS>-2.6280345916748047</m-velocityBodyV-MPS><m-velocityBodyW-MPS>-0.64850509166717529</m-velocityBodyW-MPS><m-accelerationWorldAX-MPS2>-4.3766188621520996</m-accelerationWorldAX-MPS2><m-accelerationWorldAY-MPS2>-6.6690726280212402</m-accelerationWorldAY-MPS2><m-accelerationWorldAZ-MPS2>9.2979526519775391</m-accelerationWorldAZ-MPS2><m-accelerationBodyAX-MPS2>-0.53152298927307129</m-accelerationBodyAX-MPS2><m-accelerationBodyAY-MPS2>10.025743305683136</m-accelerationBodyAY-MPS2><m-accelerationBodyAZ-MPS2>-8.8302364349365234</m-accelerationBodyAZ-MPS2><m-windX-MPS>0</m-windX-MPS><m-windY-MPS>0</m-windY-MPS><m-windZ-MPS>0</m-windZ-MPS><m-propRPM>10886.970703125</m-propRPM><m-heliMainRotorRPM>-1</m-heliMainRotorRPM><m-batteryVoltage-VOLTS>-1</m-batteryVoltage-VOLTS><m-batteryCurrentDraw-AMPS>-1</m-batteryCurrentDraw-AMPS><m-batteryRemainingCapacity-MAH>-1</m-batteryRemainingCapacity-MAH><m-fuelRemaining-OZ>11.978252410888672</m-fuelRemaining-OZ><m-isLocked>false</m-isLocked><m-hasLostComponents>false</m-hasLostComponents><m-anEngineIsRunning>true</m-anEngineIsRunning><m-isTouchingGround>true</m-isTouchingGround><m-flightAxisControllerIsActive>true</m-flightAxisControllerIsActive><m-currentAircraftStatus>CAS-FLYING</m-currentAircraftStatus></m-aircraftState><m-notifications><m-resetButtonHasBeenPressed>false</m-resetButtonHasBeenPressed></m-notifications></ReturnData></SOAP-ENV:Body></SOAP-ENV:Envelope>'
    fa = FlightAxis()
    fa.parseResponse(res)