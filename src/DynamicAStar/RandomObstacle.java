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

    @Override
    public List<Double[]> getCollisionIntervals(Vect point, double robotRadius) {
        robotRadius += AVOID_RADIUS;
        List<Double[]> collisionIntervals = new ArrayList<Double[]>();
        Double[] collisionInterval = Obstacle.getTimeIntersectCircleAroundOrigin(position.minus(point), velocity, robotRadius+radius);
        if (collisionInterval != null)
            collisionIntervals.add(collisionInterval);
        return collisionIntervals;
    }
    
    @Override
    public boolean checkIfCollidesWithRobot(Vect robotOrigin, Vect robotVelocity, double robotRadius, double startMovingTime, double endMovingTime) {
        robotRadius += AVOID_RADIUS;
        Vect relativePoint = position.minus(robotOrigin);
        Vect relativeVelocity = velocity.minus(robotVelocity);
        Double[] collisionInterval = Obstacle.getTimeIntersectCircleAroundOrigin(relativePoint, relativeVelocity, robotRadius+radius);
        if (collisionInterval != null)
            return (collisionInterval[0] <= endMovingTime && collisionInterval[1] >= startMovingTime);
        return false;
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
    private Vect getRandomVelocity() {
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
