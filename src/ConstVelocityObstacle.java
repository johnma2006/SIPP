import java.util.ArrayList;
import java.util.List;

import physics.Vect;


public class ConstVelocityObstacle {

    public double radius;
    public Vect position;
    public Vect velocity;
    private final double AVOID_RADIUS = 0.05;

    public ConstVelocityObstacle(Vect position, Vect velocity, double radius) {
        this.position = position;
        this.velocity = velocity;
        this.radius = radius;
    }

    public List<Double[]> getCollisionIntervals(Vect point, double robotRadius) {
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
}
