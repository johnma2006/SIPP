package DynamicAStar;

import java.util.List;

import physics.Vect;

public interface Obstacle {
    
    public double AVOID_RADIUS = 0.08;
    
    public List<Double[]> getCollisionIntervals(Vect point, double robotRadius);

    public void updatePosition(double time);
    
    public double getRadius();
    
    public Vect getPosition();
    
    public Vect getVelocity();
    
    public void setPosition(Vect newPosition);
    
    public void setVelocity(Vect newVelocity);
    
}
