package frc.robot.commands;
import java.nio.ByteBuffer;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;

/**
 * LED Controller bus interface class class.
 *
 * 
 * 
 *
 * 
 * 
 * 
 */

public class ledControl {

    private boolean m_confirm;
    private int m_deviceAddress;
    //private boolean m_commConfirm;
    private ByteBuffer m_bufferS;
    private ByteBuffer m_bufferR;
    private int m_redDumb;
    private int m_greenDumb;
    private int m_blueDumb;
    private byte m_byte1;
    private byte m_byte2;
    private byte m_byte3;
    private byte m_byte4;

    private final int BYTE_COUNT = 10;

    
    /**
     * Constructs the ledControl class with the Paramaters: 
     * @param confirm 
     * @param address
     * confirm(bool) tells the class to wait for a byte return of 10101010(0xAA) and will return an error if it doesnt receive within a second.
     * address(int) tells the class where to talk 
     */

    public ledControl(boolean confirm, int address) {
        m_confirm = confirm;
        m_deviceAddress = address;
        m_bufferS = ByteBuffer.allocate(BYTE_COUNT);
        m_bufferR = ByteBuffer.allocate(BYTE_COUNT);

        /*\m_commConfirm = Wire.addressOnly();
        if (m_commConfirm == true) {
            System.err.println("Err(ledControl): LEDI2C Device Not Detected.");
        }*/
    }
    I2C Wire = new I2C(Port.kMXP, 85);

    /**
     * Sets the LEDs to a solid color
     * @param red (int) 0 - 255 indicating how red the LEDs should be
     * @param green (int) 0 - 255 indicating how green the LEDs should be
     * @param blue (int) 0 - 255 indicating how blue the LEDs should be
     */
    public void setColor(int red, int green, int blue) {
        m_redDumb = Math.min(red, 255);
        m_greenDumb = Math.min(green, 255); 
        m_blueDumb = Math.min(blue, 255);
        if (red < 0 || green < 0 || blue < 0) {
            System.err.println("Err(SetColor):How?... Why?... Red, Green, Or Blue should not be negative.");
        }

        m_byte1 = 0x01; //0x01 is the "parse" byte for the Dumb Color setting

        m_byte2 = (byte) m_redDumb;
        m_byte3 = (byte) m_greenDumb;
        m_byte4 = (byte) m_blueDumb;

        boolean success = sendData(m_byte1, m_byte2, m_byte3, m_byte4);

        if (success == true) {
            return;
        } else {
            System.err.println("Err(setColor): SendData Failed. Check SendData error message.");
            return;
        }


    }

    /**
     * Alt form of SetColor that sets to a particular team color
     * @param team (String) RED or BLU (capitalization doesnt matter) / RED or BLUE if you dont get the reference 
     */
    public void setColor(String team) {

        if (team.toLowerCase() == "red") {
            setColor(255, 0, 0);
            return;
        } else if (team.toLowerCase() == "blu" || team.toLowerCase() == "blue"){
            setColor(0, 0, 255);
            return;
        } else {
            System.err.println("Err(SetColorAlt): No team was specified. Check Spelling?");
        }

    }



    /**
     * Tells the controller to start up a demo based on the demoNumber
     * @param demoNumber (int) which demo to display
     * Current Available Demos:
     * (0): Rainbow
     * (1): Firebird
     */
    public void setDemo(int demoNumber) {

        m_byte1 = 0x02; //Parse byte indicating "Demo Mode"

        switch (demoNumber) {
            case 0:
                m_byte2 = 0x01;
                break;
        
            case 1:
                m_byte2 = 0x02;
                break;

            default:
                System.err.println("Err(setDemo): demoNumber out of range. Check the function description to see what demos are available. If thats unavailable AFAIK the currently available demos are 0 and 1");
                break;
        }

        m_byte3 = (byte) 0xAA;
        m_byte4 = (byte) 0xAA;

        boolean success = sendData(m_byte1, m_byte2, m_byte3, m_byte4);

        if (success == true) {
            return;
        } else {
            System.err.println("Err(setDemo): SendData Failed. Check SendData error message.");
            return;
        }

    }



    /**
     * A tentative implementation of a "Game Event" System. 
     * @param gameEvent (int) Which "Game Event" has happened. Current "Events":
     * (0): Flashes green for a second to indicate a note has been picked up.
     * (1)
     */
    public void setGameEvent(int gameEvent) {

        m_byte1 = 0x03; //Parse byte indicating "Game Event Mode"

        switch (gameEvent) {
            case 0:
                m_byte2 = 0x01;
                break;
            
            default:
                System.err.println("Err(setDemo): demoNumber out of range. Check the function description to see what demos are available. If thats unavailable AFAIK the currently available Game Events are 0");
                break;
        }

        boolean success = sendData(m_byte1, m_byte2, m_byte3, m_byte4);

        if (success == true) {
            return;
        } else {
            System.err.println("Err(setDemo): SendData Failed. Check SendData error message.");
            return;
        }

    }

    
     
    /**
     * Togglable remote shutoff. Using it once will shut off the lights, Using it again will restore the lights.
     * 
     */
    public void reset() {
        m_byte1 = 0x05;
        m_byte2 = (byte) 0xAA;
        m_byte3 = (byte) 0xAA;
        m_byte4 = (byte) 0xAA;
    }


    private boolean sendData(byte byte1, byte byte2, byte byte3, byte byte4){
        

        m_bufferS.put(0, (byte) byte1);
        m_bufferS.put(1, (byte) byte2);
        m_bufferS.put(2, (byte) byte3);
        m_bufferS.put(3, (byte) byte4);
        if (m_confirm == false){

            Wire.writeBulk(m_bufferS, 4);

            byte1 = 0x00;
            byte2 = 0x00;
            byte3 = 0x00;
            byte4 = 0x00;
            clearBuffers();

            return true;

        } else {
            Wire.transaction(m_bufferS, 4, m_bufferR, 1);
            if (m_bufferR.get(0) == 0xAA) {
                return true;
            } else {
                System.err.println("Err(SendData): LEDI2C Response Not Detected. Is the device set to confirm?");
                byte1 = 0x00;
                byte2 = 0x00;
                byte3 = 0x00;
                byte4 = 0x00;
                clearBuffers();
                return false;
            }

        }   
    }

    private void clearBuffers(){
        for(int i = 0; i < BYTE_COUNT; i++){
            m_bufferR.put(i, (byte) 0x00);
            m_bufferS.put(i, (byte) 0x00);
        }
        
        m_byte1 = 0x00;
        m_byte2 = 0x00;
        m_byte3 = 0x00;
        m_byte4 = 0x00;

    }

    }

