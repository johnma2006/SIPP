package DynamicAStar;

import java.util.List;

import physics.Vect;

public interface Obstacle {
    
    public double AVOID_RADIUS = 0.08;
    
    public static Double[] getTimeIntersectCircleAroundOrigin(Vect position, Vect velocity, double radius) {
        Vect origin = new Vect(0, 0);
        Vect projection = origin.minus(position).projectOn(velocity.unitSize()).plus(position);
        double lengthSquared = radius * radius - projection.distanceSquared(origin);
        if (lengthSquared <= 0) // does not intersect
            return null;

        double distanceToProjection = projection.minus(position).length();
        if (projection.minus(position).dot(velocity) < 0)
            distanceToProjection *= -1;
        double length = Math.sqrt(lengthSquared);
        Double[] collisionInterval = new Double[]{(distanceToProjection-length) / velocity.length(),
                (distanceToProjection+length) / velocity.length()};
        return collisionInterval;
    }
    
    public List<Double[]> getCollisionIntervals(Vect point, double robotRadius);
    
    public boolean checkIfCollidesWithRobot(Vect robotOrigin, Vect robotVelocity, double robotRadius, double startMovingTime, double endMovingTime);
    
    public void updatePosition(double time);
    
    public double getRadius();
    
    public Vect getPosition();
    
    public Vect getVelocity();
    
    public void setPosition(Vect newPosition);
    
    public void setVelocity(Vect newVelocity);
    
}
