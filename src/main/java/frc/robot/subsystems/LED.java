package frc.robot.subsystems;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase{
    private final AddressableLED m_led = new AddressableLED(0);
    private final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(28);
    private int yellow_dot = 0;
    private int brown_dot = 0;
    private boolean warning = false;
    private final Intake m_intake;
    public boolean pathfinding = false;
    public boolean operatorPathfinding = false;
    public boolean boomPathfinding = false;
    public int pulseOffset = 0;
    public double matchTime;
    public boolean robotOrient;
    public double rainbow_dot;
    public boolean bottomLimitDot;
    public LED(Intake intake){
        m_intake = intake;
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();
        set(0, 0, 0);
    }


    public void set(int red, int green, int blue){
        for (int i=0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, red, green, blue);
            
        }
    }
    public void setPulseRed(){
        int intensity = 255 - pulseOffset;
            for (int i = 0; i < m_ledBuffer.getLength() / 2 + 1; i++) {
                m_ledBuffer.setRGB(i, (intensity + 25*i) % 255, 0, 0);
                m_ledBuffer.setRGB(m_ledBuffer.getLength()-1-i, (intensity + 25*i) % 255, 0, 0);
            }
            pulseOffset = (pulseOffset + 15) % 255;
    }

    public void setPulseBlue(){
        int intensity = 255 - pulseOffset;
            for (int i = 0; i < m_ledBuffer.getLength() / 2 + 1; i++) {
                m_ledBuffer.setRGB(i, 0, 0, (intensity + 25*i) % 255);
                m_ledBuffer.setRGB(m_ledBuffer.getLength()-1-i, 0, 0, (intensity + 25*i) % 255);
            }
            pulseOffset = (pulseOffset + 15) % 255;
    }

    public void yellow_dot(){
        yellow_dot++;
        for(int i=0;i<m_ledBuffer.getLength(); i++){
            if(yellow_dot%28==i){
                m_ledBuffer.setRGB(i, 255, 255, 0);
            } else {
                m_ledBuffer.setRGB(i, 0, 0, 255);
            }
        }
    }

      public void purple_dot(){
        yellow_dot++;
        for(int i=0;i<m_ledBuffer.getLength(); i++){
            if(yellow_dot%28==i){
                m_ledBuffer.setRGB(i, 255, 255, 0);
            } else {
                m_ledBuffer.setRGB(i, 160, 32, 240);
            }
        }
    }

    public void rainbow_dot(){
        rainbow_dot++;

        for(int i=0;i<m_ledBuffer.getLength(); i++){
            if(i%7==0){
                m_ledBuffer.setRGB(i, 255,0,0);
            }else if(i%7==1){
                m_ledBuffer.setRGB(i, 255, 165, 0);
            }else if(i%7==2){
                m_ledBuffer.setRGB(i, 255, 255, 0);
            }else if(i%7==3){
                m_ledBuffer.setRGB(i, 0, 255, 0);
            }else if(i%7==4){
                m_ledBuffer.setRGB(i,0,0,255);
            } else if(i%7==5) {
                m_ledBuffer.setRGB(i, 75, 0, 130);
            } else if(i%7==6) {
                m_ledBuffer.setRGB(i, 127, 0, 255);
            }
        }
        // Orange
        // set(255,165,0);

        //Yellow
        // set(255,255,0);

        //Green
        // set(0,255,0);

        //Blue
        // set(0,0,255);

        //indigo
        // set(75,0,130);

        //violet
        // set(127,0,255);
    }
    
      public void realYellow_dot(){
        yellow_dot++;
        for(int i=0;i<m_ledBuffer.getLength(); i++){
            if(yellow_dot%28==i){
                m_ledBuffer.setRGB(i, 255, 255, 255);
            } else {
                m_ledBuffer.setRGB(i, 255, 255, 0);
            }
        }
    }
    public void bottomLimit(){
        for(int i = 0; i<m_ledBuffer.getLength(); i++){
            if(i%4 <2){
                m_ledBuffer.setRGB(i, 160, 32, 240);
            }else{
                m_ledBuffer.setRGB(i, 255,0,0);
            }
        }
    }


    public void idle() {
        var alliance = DriverStation.getAlliance();
        if(!alliance.isPresent()){
            return;
        }

        if (alliance.get()== DriverStation.Alliance.Blue) {
            setPulseBlue();
        }
        else if(alliance.get() == DriverStation.Alliance.Red) {
            setPulseRed();
        }
    }
    @Override
    public void periodic() {
        if(!DriverStation.isEnabled()){
        idle();
        } else {
            matchTime = Timer.getMatchTime();
            if(matchTime>20||matchTime<18){
                if(pathfinding){
                 yellow_dot();
                } else if(operatorPathfinding) {
                purple_dot();
                }  else if(robotOrient){
                    rainbow_dot();
                }else if (boomPathfinding) {
                    realYellow_dot();
                }
                else if(bottomLimitDot){
                    bottomLimit();
                }
                else if(m_intake.getIRsensor()){
                set(255, 0, 0);
                } else {
                set(0, 255, 0);
             }
           
             } else {
                if(warning){
                    set(255, 50, 50);
                } else {
                    set(0, 0, 0);
                }
            warning = !warning;
        }

    }
    
        m_led.setData(m_ledBuffer);
    }
}