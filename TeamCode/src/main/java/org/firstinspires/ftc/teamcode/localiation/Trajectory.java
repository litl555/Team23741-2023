package org.firstinspires.ftc.teamcode.localiation;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class Trajectory {
    Pose2d start,end,startVelo,endVelo,startAccel,endAccel,p0,p1,p2,p3,p4,p5;
    public double length=0;

    public Trajectory(Pose2d start, Pose2d end, Pose2d startVelo, Pose2d endVelo, Pose2d startAccel, Pose2d endAccel) {
        this.start = start;
        this.end = end;
        this.startVelo = startVelo;
        this.endVelo = endVelo;
        this.startAccel = startAccel;
        this.endAccel = endAccel;
        p0=start;
        p5=end;
        p1=new Pose2d(startVelo.getX()/5.0+p0.getX(),startVelo.getY()/5.0+p0.getY());
        p2=new Pose2d(startAccel.getX()/20.0+2.0*p1.getX()-p0.getX(),startAccel.getY()/20.0+2.0*p1.getY()-p0.getY());
        p4=new Pose2d(p5.getX()-endVelo.getX()/5.0,p5.getY()-endVelo.getY()/5.0);
        p3=new Pose2d(endAccel.getX()/20.0+2.0*p4.getX()-p5.getX(),endAccel.getY()/20.0+2.0*p4.getY()-p5.getY());
        length=calculateLength();
    }
    private double calculateLength(){
        double integral=0;
        for(int i=0;i<100;i++){
            Pose2d velocity=velocities(i/100.0);
            integral+=1.0/100.0*(Math.sqrt(Math.pow(velocity.getX(),2)+Math.pow(velocity.getY(),2)));
        }
        return(integral);
    }
    private Pose2d equation(double t){
        return(new Pose2d(    Math.pow((1.0-t),5.0)*p0.getX() +   Math.pow((1.0-t),4)*t*5.0*p1.getX()   +   Math.pow((1.0-t),3)*t*t*10.0*p2.getX()  +   Math.pow((1.0-t),2)*t*t*t*10.0*p3.getX()  +   Math.pow((1.0-t),1)*t*t*t*t*5.0*p4.getX()   +   Math.pow(t,5)*p5.getX(),Math.pow((1.0-t),5.0)*p0.getY() +   Math.pow((1.0-t),4)*t*5.0*p1.getY()   +   Math.pow((1.0-t),3)*t*t*10.0*p2.getY()  +   Math.pow((1.0-t),2)*t*t*t*10.0*p3.getY()  +   Math.pow((1.0-t),1)*t*t*t*t*5.0*p4.getY()   +   Math.pow(t,5)*p5.getY()));
    }
    private Pose2d velocities(double t){
        return(new Pose2d(5.0*((p5.getX()-5.0*p4.getX()+10.0*p3.getX()-10.0*p2.getX()+5.0*p1.getX()-p0.getX())*Math.pow(t,4)   +  4.0*(p4.getX()-4.0*(p3.getX()+p1.getX())+6.0*p2.getX()+p0.getX())*Math.pow(t,3)  +   6.0*(p3.getX()-3.0*p2.getX()+3.0*p1.getX()-p0.getX())*t*t+4.0*(p2.getX()-2.0*p1.getX()+p0.getX())*t+p1.getX()-p0.getX()),5.0*((p5.getY()-5.0*p4.getY()+10.0*p3.getY()-10.0*p2.getY()+5.0*p1.getY()-p0.getY())*Math.pow(t,4)   +  4.0*(p4.getY()-4.0*(p3.getY()+p1.getY())+6.0*p2.getY()+p0.getY())*Math.pow(t,3)  +   6.0*(p3.getY()-3.0*p2.getY()+3.0*p1.getY()-p0.getY())*t*t+4.0*(p2.getY()-2.0*p1.getY()+p0.getY())*t+p1.getY()-p0.getY())));
    }

}
