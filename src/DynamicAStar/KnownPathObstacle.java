package DynamicAStar;
import java.util.ArrayList;
import java.util.List;

import physics.Vect;


public class KnownPathObstacle implements Obstacle {

    public double radius;
    public List<Node> path;

    public KnownPathObstacle(List<Node> path, double radius) {
        this.radius = radius;
        this.path = path;
    }

    public List<Double[]> getCollisionIntervals(Vect point, double robotRadius) {
        robotRadius += AVOID_RADIUS;
        List<Double[]> collisionIntervals = new ArrayList<Double[]>();
        
        for (int i = 0; i < path.size() - 1; i++) {
            Vect position = path.get(i).position;
            Vect velocity = path.get(i+1).position.minus(path.get(i).position).times(1.0 / (path.get(i+1).time - path.get(i).time));
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
            double t1 = Math.max(collisionInterval[0] + path.get(i).time, path.get(i).time);
            double t2 = Math.min(collisionInterval[1] + path.get(i).time, path.get(i+1).time);
            if (t2 > t1) {
                if (collisionIntervals.size() > 0)
                    if (collisionIntervals.get(collisionIntervals.size() - 1)[1] == t1)
                        t1 = collisionIntervals.remove(collisionIntervals.size() - 1)[0];
                collisionIntervals.add(new Double[]{t1, t2});
            }
        }
        return collisionIntervals;
    }

    @Override
    public void updatePosition(double time) {
        
    }

    @Override
    public double getRadius() {
        return this.radius;
    }

    @Override
    public Vect getPosition() {
        return null;
    }

    @Override
    public Vect getVelocity() {
        return null;
    }

    @Override
    public void setPosition(Vect newPosition) {
        
    }

    @Override
    public void setVelocity(Vect newVelocity) {
        
    }
}
