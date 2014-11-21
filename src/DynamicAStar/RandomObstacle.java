package DynamicAStar;
import java.util.ArrayList;
import java.util.List;

import physics.Vect;


public class RandomObstacle implements Obstacle {

    public double radius;
    public Vect position;
    public Vect velocity;
    public double timeUntilChangeVelocity;
    private final double MAX_VELOCITY = 1;
    private final double MAX_TIME_TILL_CHANGE_VELOCITY = 1.5;

    public RandomObstacle(Vect position, double radius) {
        this.radius = radius;
        this.position = position;
        this.velocity = getRandomVelocity();
        this.timeUntilChangeVelocity = Math.random() * MAX_TIME_TILL_CHANGE_VELOCITY;
    }

    public List<Double[]> getCollisionIntervals(Vect point, double robotRadius) {
        robotRadius += AVOID_RADIUS;
        List<Double[]> collisionIntervals = new ArrayList<Double[]>();

        Vect projection = point.minus(position).projectOn(velocity.unitSize()).plus(position);
        double lengthSquared = (radius+robotRadius)*(radius+robotRadius) - 
                (projection.distanceSquared(point));
        if (lengthSquared <= 0) // does not intersect
            return collisionIntervals;
        
        double distanceToProjection = projection.minus(position).length();
        if (projection.minus(position).dot(velocity) < 0)
            distanceToProjection *= -1;
        double length = Math.sqrt(lengthSquared);
        Double[] collisionInterval = new Double[]{(distanceToProjection-length) / velocity.length(),
                (distanceToProjection+length) / velocity.length()};
        collisionIntervals.add(collisionInterval);
        return collisionIntervals;
    }
    
    @Override
    public void updatePosition(double time) {
        this.position = this.position.plus(this.velocity.times(time));
        timeUntilChangeVelocity -= time;
        if (timeUntilChangeVelocity < 0) {
            this.velocity = getRandomVelocity();
            this.timeUntilChangeVelocity = Math.random() * MAX_TIME_TILL_CHANGE_VELOCITY;
        }
    }
    public Vect getRandomVelocity() {
        double magnitude = Math.random() * MAX_VELOCITY;
        double angle = Math.random() * 2 * Math.PI;
        return new Vect(Math.cos(angle), Math.sin(angle)).times(magnitude);
    }
    
    @Override
    public double getRadius() {
        return this.radius;
    }
    
    @Override
    public Vect getPosition() {
        return this.position;
    }
    
    @Override
    public Vect getVelocity() {
        return this.velocity;
    }
    
    @Override
    public void setPosition(Vect newPosition) {
        this.position = newPosition;
    }

    @Override
    public void setVelocity(Vect newVelocity) {
        this.velocity = newVelocity;
    }
    
}
