package DynamicAStar;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Set;

import javax.swing.JFrame;
import javax.swing.JPanel;

import physics.Vect;


public class DynamicPathPlanner extends JPanel {

    private final static double SCALING_FACTOR = 300; // for drawing the field

    private final static double ROBOT_RADIUS = 0.2;
    private final static double ROBOT_VELOCITY = 1; // 1 meter per second
    private final static double GRID_STEP_SIZE = 0.1; // 0.1 meter

    private final static double FIELD_LENGTH = 6;
    private final static double FIELD_WIDTH = 4;

    private final static double MIN_X = 0;
    private final static double MAX_X = FIELD_LENGTH;
    private final static double MIN_Y = 0;
    private final static double MAX_Y = FIELD_WIDTH;


    public final List<Obstacle> obstacles;

    public static void main(String[] args) {

        JFrame frame = new JFrame("Dynamic Path Planner");

        DynamicPathPlanner pp = new DynamicPathPlanner();

        //        pp.obstacles.add(new RandomObstacle(new Vect(1, 1), ROBOT_RADIUS));
        //        pp.obstacles.add(new RandomObstacle(new Vect(3, 1), ROBOT_RADIUS));
        //        pp.obstacles.add(new RandomObstacle(new Vect(5, 1), ROBOT_RADIUS));
        //        pp.obstacles.add(new RandomObstacle(new Vect(2, 2), ROBOT_RADIUS));
        //        pp.obstacles.add(new RandomObstacle(new Vect(4, 2), ROBOT_RADIUS));
        //        pp.obstacles.add(new RandomObstacle(new Vect(3, 3), ROBOT_RADIUS));
        //        pp.obstacles.add(new RandomObstacle(new Vect(5, 3), ROBOT_RADIUS));

        pp.obstacles.add(new ConstVelocityObstacle(new Vect(4, 0), new Vect(-1, 1), ROBOT_RADIUS));
        pp.obstacles.add(new ConstVelocityObstacle(new Vect(4, 1), new Vect(-0.6, 0.6), ROBOT_RADIUS));
        pp.obstacles.add(new ConstVelocityObstacle(new Vect(6, 2.1), new Vect(-1, 0), ROBOT_RADIUS));
        pp.obstacles.add(new ConstVelocityObstacle(new Vect(6, 2.5), new Vect(-0.75, 0), ROBOT_RADIUS));
        pp.obstacles.add(new ConstVelocityObstacle(new Vect(6, 2.9), new Vect(-0.5, 0), ROBOT_RADIUS));
        pp.obstacles.add(new ConstVelocityObstacle(new Vect(0, 1), new Vect(0.78, 0), ROBOT_RADIUS));
        pp.obstacles.add(new ConstVelocityObstacle(new Vect(3.5, 2.7), new Vect(0, 0), ROBOT_RADIUS));


        pp.getPath(new Vect(1, 3), new Vect(5, 1), ROBOT_RADIUS);
        System.out.println("done");
        
        frame.add(pp);
        frame.setSize((int)(1827. / 300 * SCALING_FACTOR), (int)(1270. / 300 * SCALING_FACTOR));
        frame.setVisible(true);
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e1) {
            e1.printStackTrace();
        }
        while (true) {
            pp.run(new Vect(1, 3.5), new Vect(5, 1), ROBOT_RADIUS);
            pp.run(new Vect(5, 1), new Vect(1, 3.5), ROBOT_RADIUS);
        }
    }

    /* 
     * ===================================================================================
     *                                  Path planner
     * ===================================================================================
     */


    public DynamicPathPlanner() {
        obstacles = new ArrayList<Obstacle>();
        robots = new ArrayList<Obstacle>();
    }

    public List<Node> getPath(Vect start, Vect goal, double robotRadius) {     
        Double TIMEOUT = 1.0;
        long initialTime = System.nanoTime();
        PriorityQueue<Node> open = new PriorityQueue<Node>(new Comparator<Node>(){
            public int compare(Node n1, Node n2) {
                return Double.compare(n1.time+heuristic(n1.position, goal), 
                        n2.time+heuristic(n2.position, goal));
            }
        });
        double totalTime = System.nanoTime();
        Set<Node> expanded = new HashSet<Node>();
        Double[] initialSafeInterval = getSafeIntervals(start, robotRadius).get(0); // TODO: MAKE SURE DOESN'T THROW EXCEPTION!
        open.add(new Node(start, initialSafeInterval, 0));

        while (!open.isEmpty()) {
            if (System.nanoTime() - initialTime > 1000000000 * TIMEOUT)
                return new ArrayList<Node>();
            Node next = open.poll();
            if (equals(next.position, goal)) {
                List<Node> path = new ArrayList<Node>();
                while (next != null) {
                    path.add(next);
                    next = next.parent;
                }
                Collections.reverse(path);
                System.out.println("Took " + (System.nanoTime() - totalTime)/1000000000 + "s");
                System.out.println("Expanded " + expanded.size() + " nodes\nPath length: " + path.size());
                return path;
            }
            if (!expanded.contains(next)) {
                expanded.add(next);
                List<Node> successors = getSuccessors(next, robotRadius);
                for (Node neighbour : successors) {
                    if (!expanded.contains(neighbour)) {
                        neighbour.parent = next;
                        open.add(neighbour);
                    }
                }
            }
        }
        return new ArrayList<Node>(); // Did not find path
    }

    public double heuristic(Vect point, Vect goal) {
        return 1.2 * point.minus(goal).length() / ROBOT_VELOCITY;
    }

    public static boolean equals(Vect v1, Vect v2) {
        double EPS = 0.1;
        if (Math.abs(v1.x() - v2.x()) > EPS)
            return false;
        if (Math.abs(v1.y() - v2.y()) > EPS)
            return false;
        return true;
    }

    public List<Node> getSuccessors(Node node, double robotRadius) {
        List<Node> successors = new ArrayList<Node>();
        List<Vect> motions = new ArrayList<Vect>();
        for (double x = -1.0; x <= 1.0; x+=0.5)
            for (double y = -1.0; y <= 1.0; y+=0.5)
                if (x == -1 || x == 1 || y == -1 || y == 1)
                    motions.add(new Vect(x*GRID_STEP_SIZE, y*GRID_STEP_SIZE));

        for (Vect m : motions) {
            Vect point = new Vect(node.position.x()+m.x(), node.position.y()+m.y());
            if (point.x() >= MIN_X && point.x() <= MAX_X && point.y() >= MIN_Y && point.y() <= MAX_Y) {
                double motionTime = m.length() / ROBOT_VELOCITY;
                double startTime = node.time + motionTime;
                double endTime = node.safeInterval[1] + motionTime;
                for (Double[] s : getSafeIntervals(point, robotRadius)) {
                    if (s[0] <= endTime && s[1] >= startTime) {
                        double time = Math.max(startTime, s[0]); // earliest time without collisions
                        Node neighbour = new Node(point, s, time);
                        successors.add(neighbour);
                    }
                }
            }
        }
        return successors;
    }

    public List<Double[]> getSafeIntervals(Vect point, double robotRadius) {
        List<Double[]> safeIntervals = new ArrayList<Double[]>();

        List<Double[]> allCollisionIntervals = new ArrayList<Double[]>();
        for (Obstacle obs : obstacles)
            allCollisionIntervals.addAll(obs.getCollisionIntervals(point, robotRadius));
        Collections.sort(allCollisionIntervals, new Comparator<Double[]>() {
            public int compare(Double[] d1, Double[] d2) {return Double.compare(d1[0], d2[0]);}
        });

        List<Double[]> mergedCollisionIntervals = new ArrayList<Double[]>();
        if (allCollisionIntervals.size() > 0) {
            double start = allCollisionIntervals.get(0)[0];
            double end = allCollisionIntervals.get(0)[1];
            for (int i = 1; i < allCollisionIntervals.size(); i++) {
                if (allCollisionIntervals.get(i)[0] > end) {
                    mergedCollisionIntervals.add(new Double[]{start, end});
                    start = allCollisionIntervals.get(i)[0];
                }
                end = Math.max(end, allCollisionIntervals.get(i)[1]);
            }
            mergedCollisionIntervals.add(new Double[]{start, end});
        }

        List<Double> safeIntervalTimes = new ArrayList<Double>();
        safeIntervalTimes.add(Double.NEGATIVE_INFINITY);
        for (Double[] d : mergedCollisionIntervals) {
            safeIntervalTimes.add(d[0]);
            safeIntervalTimes.add(d[1]);
        }
        safeIntervalTimes.add(Double.POSITIVE_INFINITY);
        for (int i = 0; i < safeIntervalTimes.size() - 1; i += 2) {
            if (!safeIntervalTimes.get(i).equals(safeIntervalTimes.get(i+1)) && safeIntervalTimes.get(i+1) > 0)
                safeIntervals.add(new Double[]{safeIntervalTimes.get(i), safeIntervalTimes.get(i+1)});
        }
        return safeIntervals;
    }



    /* 
     * ===================================================================================
     *                                  Drawing functions
     * ===================================================================================
     */

    public final List<Obstacle> robots;
    public List<Node> path = new ArrayList<Node>();

    public void run(Vect start, Vect goal, double robotRadius) {
        double time = 0;
        double TIMESTEP = 1./30;
        this.path = getPath(start, goal, robotRadius);
        Vect currentPosition = start;
        double frameTime = System.nanoTime();
        while (true) {
            updateRobotPositions(this.path, time, robotRadius);
            updateObstaclePositions(TIMESTEP);
            this.repaint();
            time = TIMESTEP;
            if (robots.size() > 0)
                currentPosition = robots.get(0).getPosition();
            else
                if (currentPosition.minus(goal).length() < 0.15)
                    break;
            List<Node> newPath = getPath(currentPosition, goal, robotRadius);
            if (newPath.size() > 0)
                this.path = newPath;
            while ((System.nanoTime() - frameTime)/1000000000 < TIMESTEP) {}
            frameTime = System.nanoTime();
        }
    }

    public void updateRobotPositions(List<Node> path, double time, double robotRadius) {
        while (!robots.isEmpty())
            robots.remove(robots.size()-1);
        for (int i = 0; i < path.size(); i++) {
            if (path.get(i).time > time) {
                double t2 = path.get(i).time;
                Vect position1 = path.get(i-1).position;
                Vect position2 = path.get(i).position;
                Vect position;
                double travelTime = position2.minus(position1).length() / ROBOT_VELOCITY;
                if (t2 - travelTime > time)
                    position = position1;
                else {
                    double timeTravelled = time-t2+travelTime;
                    position = position1.times(travelTime-timeTravelled).plus(position2.times(timeTravelled)).times(1/travelTime);
                }
                Obstacle robot = new ConstVelocityObstacle(position, new Vect(0, 0), robotRadius);
                robots.add(robot);
                break;
            }
        }
    }

    public void updateObstaclePositions(double time) {
        for (Obstacle obs : obstacles) {
            obs.updatePosition(time);
            if (obs.getPosition().x() < MIN_X + obs.getRadius()) {
                obs.setVelocity(new Vect(-obs.getVelocity().x(), obs.getVelocity().y()));
                obs.setPosition(new Vect(MIN_X + obs.getRadius(), obs.getPosition().y()));
            }
            if (obs.getPosition().x() > MAX_X - obs.getRadius()) {
                obs.setVelocity(new Vect(-obs.getVelocity().x(), obs.getVelocity().y()));
                obs.setPosition(new Vect(MAX_X - obs.getRadius(), obs.getPosition().y()));
            }
            if (obs.getPosition().y() < MIN_Y + obs.getRadius()) {
                obs.setVelocity(new Vect(obs.getVelocity().x(), -obs.getVelocity().y()));
                obs.setPosition(new Vect(obs.getPosition().x(), MIN_Y + obs.getRadius()));
            }
            if (obs.getPosition().y() > MAX_Y - obs.getRadius()) {
                obs.setVelocity(new Vect(obs.getVelocity().x(), -obs.getVelocity().y()));
                obs.setPosition(new Vect(obs.getPosition().x(), MAX_Y - obs.getRadius()));
            }
        }
    }

    @Override
    public void paint(Graphics g) {
        super.paint(g);
        Graphics2D g2d = (Graphics2D) g;
        int offset = 0;

        // Grid
        //        for (double x = 0; x <= FIELD_LENGTH+0.01; x += GRID_STEP_SIZE)
        //            for (double y = 0; y <= FIELD_WIDTH+0.01; y += GRID_STEP_SIZE)
        //                g2d.fillOval((int)(x*SCALING_FACTOR)+offset, (int)(y*SCALING_FACTOR)+offset, 5, 5);
        

        for (Obstacle obs : obstacles) {
            g.setColor(Color.DARK_GRAY);
            g2d.fillOval((int)((obs.getPosition().x()-obs.getRadius())*SCALING_FACTOR)+offset, (int)((obs.getPosition().y()-obs.getRadius())*SCALING_FACTOR)+offset, 
                    (int)(2*obs.getRadius()*SCALING_FACTOR), (int)(2*obs.getRadius()*SCALING_FACTOR));
        }

        for (Obstacle obs : robots) {
            g.setColor(Color.LIGHT_GRAY);
            g2d.fillOval((int)((obs.getPosition().x()-obs.getRadius())*SCALING_FACTOR)+offset, (int)((obs.getPosition().y()-obs.getRadius())*SCALING_FACTOR)+offset, 
                    (int)(2*obs.getRadius()*SCALING_FACTOR), (int)(2*obs.getRadius()*SCALING_FACTOR));
        }

        for (int i = 0; i < path.size() - 1; i++) {
            g.setColor(Color.blue);
            g2d.drawLine((int)(path.get(i).position.x()*SCALING_FACTOR)+offset, 
                    (int)(path.get(i).position.y()*SCALING_FACTOR)+offset, 
                    (int)(path.get(i+1).position.x()*SCALING_FACTOR)+offset, 
                    (int)(path.get(i+1).position.y()*SCALING_FACTOR)+offset); 
        }
    }
}
