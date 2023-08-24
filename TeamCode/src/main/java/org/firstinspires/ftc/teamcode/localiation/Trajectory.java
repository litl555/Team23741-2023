package org.firstinspires.ftc.teamcode.localiation;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.Const;

import java.util.ArrayList;
import java.util.Dictionary;
import java.util.List;

public class Trajectory {
    Pose2d start,end,startVelo,endVelo,startAccel,endAccel,p0,p1,p2,p3,p4,p5;
    public double length=0;
    private double deltaT=.1;
    public ArrayList<Double> tValues;

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
        length=calculateLength(0,1);
        generateTValues();
    }
    private void generateTValues(){
        ArrayList<Double> values=new ArrayList<>();
        double previous=0;
        for(int i=1;i<getTotalTime()/deltaT;i++){
            values.add(getTValue(i*deltaT,previous));
            previous=values.get(values.size()-1);

        }
        int counter=0;
        for(double i:values){
            values.set(counter,values.get(counter)/values.get(values.size()-1));
            counter++;
        }
        tValues=values;

    }
    private double calculateLength(double a,double b){
        double integral=0;
        for(int i=0;i<100;i++){
            Pose2d velocity=velocities((double)i*(b-a)/100.0+a);
            integral+=1.0*(b-a)/100.0*(Math.sqrt(Math.pow(velocity.getX(),2)+Math.pow(velocity.getY(),2)));
        }
        return(integral);
    }
    private double getTValue(double time,double previousT){
        double arcLengthVelo=1.0/2.0*((getVelocityProfile(time).getX()-getVelocityProfile(time-deltaT).getX())/(deltaT))*Math.pow(deltaT,2)+getVelocityProfile(time-deltaT).getX()*deltaT;
        Constants.yes+=arcLengthVelo;
        double X=previousT+.01;
        for(int i=0;i<10;i++){
            double len=calculateLength(previousT,X);
            double slope=getSlope(X);
            X=(-1.0*(len-arcLengthVelo)+X*slope)/slope;
        }
        return X;
    }
    private double getTotalTime(){
        return((length-1.0/2.0*Constants.maxAcceleration*Math.pow(Constants.maxVelocty/Constants.maxAcceleration,2)-(1.0/2.0*Constants.maxAcceleration*Math.pow(Constants.maxVelocty/Constants.maxAcceleration,2)))/Constants.maxVelocty+2.0*(Constants.maxVelocty/Constants.maxAcceleration));
    }
    public Pose2d normalize(Pose2d vec){
        double len=Math.sqrt(Math.pow(vec.getX(),2)+Math.pow(vec.getY(),2));
        return(vec.div(len));
    }
    public Pose2d getVelocityProfile(double time){
        double totalTime=getTotalTime();
        Constants.timed=totalTime;
        if(time<Constants.maxVelocty/Constants.maxAcceleration){
            return(new Pose2d(time*Constants.maxAcceleration,Constants.maxAcceleration));
        }
        else if(time<totalTime- Constants.maxVelocty/Constants.maxAcceleration){
            return(new Pose2d(Constants.maxVelocty,0));
        }
        else if(time<totalTime){
            return(new Pose2d(-1.0*Constants.maxAcceleration*((time-(totalTime-Constants.maxVelocty/Constants.maxAcceleration)))+Constants.maxVelocty,-Constants.maxAcceleration));
        }
        else{
            return new Pose2d(0,0);

        }    }
    private double getSlope(double t){
        Pose2d velocities=velocities(t);
        return(Math.sqrt(Math.pow(velocities.getX(),2)+Math.pow(velocities.getY(),2)));
    }
    public Pose2d equation(double t){
        return(new Pose2d(    Math.pow((1.0-t),5.0)*p0.getX() +   Math.pow((1.0-t),4)*t*5.0*p1.getX()   +   Math.pow((1.0-t),3)*t*t*10.0*p2.getX()  +   Math.pow((1.0-t),2)*t*t*t*10.0*p3.getX()  +   Math.pow((1.0-t),1)*t*t*t*t*5.0*p4.getX()   +   Math.pow(t,5)*p5.getX(),Math.pow((1.0-t),5.0)*p0.getY() +   Math.pow((1.0-t),4)*t*5.0*p1.getY()   +   Math.pow((1.0-t),3)*t*t*10.0*p2.getY()  +   Math.pow((1.0-t),2)*t*t*t*10.0*p3.getY()  +   Math.pow((1.0-t),1)*t*t*t*t*5.0*p4.getY()   +   Math.pow(t,5)*p5.getY()));
    }
    public Pose2d velocities(double t){
        return(new Pose2d(5.0*((p5.getX()-5.0*p4.getX()+10.0*p3.getX()-10.0*p2.getX()+5.0*p1.getX()-p0.getX())*Math.pow(t,4)   +  4.0*(p4.getX()-4.0*(p3.getX()+p1.getX())+6.0*p2.getX()+p0.getX())*Math.pow(t,3)  +   6.0*(p3.getX()-3.0*p2.getX()+3.0*p1.getX()-p0.getX())*t*t+4.0*(p2.getX()-2.0*p1.getX()+p0.getX())*t+p1.getX()-p0.getX()),5.0*((p5.getY()-5.0*p4.getY()+10.0*p3.getY()-10.0*p2.getY()+5.0*p1.getY()-p0.getY())*Math.pow(t,4)   +  4.0*(p4.getY()-4.0*(p3.getY()+p1.getY())+6.0*p2.getY()+p0.getY())*Math.pow(t,3)  +   6.0*(p3.getY()-3.0*p2.getY()+3.0*p1.getY()-p0.getY())*t*t+4.0*(p2.getY()-2.0*p1.getY()+p0.getY())*t+p1.getY()-p0.getY())));
    }
    public Pose2d accelerrations(double t){
        return(new Pose2d(20.0*(p5.getX()-5.0*p4.getX()+10*p3.getX()-10*p2.getX()+5*p1.getX()-p0.getX())*t*t*t    +    60.0*(p4.getX()-4.0*(p3.getX()+p1.getX())+6.0*p2.getX()+p0.getX())*t*t    +    60.0*(p3.getX()-3.0*p2.getX()+3.0*p1.getX()-p0.getX())*t    +    20.0*(p2.getX()-2.0*p1.getX()+p0.getX()),20.0*(p5.getY()-5.0*p4.getY()+10*p3.getY()-10*p2.getY()+5*p1.getY()-p0.getY())*t*t*t    +    60.0*(p4.getY()-4.0*(p3.getY()+p1.getY())+6.0*p2.getY()+p0.getY())*t*t    +    60.0*(p3.getY()-3.0*p2.getY()+3.0*p1.getY()-p0.getY())*t    +    20.0*(p2.getY()-2.0*p1.getY()+p0.getY())

        ));
    }

}
