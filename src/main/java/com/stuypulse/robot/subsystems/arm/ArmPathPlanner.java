package com.stuypulse.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.*;

public class ArmPathPlanner {
    private final ArmConfigurationSpace configSpace;
    private final double error = 0.1; // Radians

    // A* Node class
    private static class Node implements Comparable<Node> {
        final Rotation2d theta1, theta2;
        double gScore = Double.POSITIVE_INFINITY;
        double fScore = Double.POSITIVE_INFINITY;
        Node parent = null;

        Node(Rotation2d theta1, Rotation2d theta2) {
            this.theta1 = theta1;
            this.theta2 = theta2;
        }

        @Override
        public int compareTo(Node other) {
            return Double.compare(this.fScore, other.fScore);
        }
    }

    public ArmPathPlanner(ArmConfigurationSpace configSpace) {
        this.configSpace = configSpace;
    }

    public List<Translation2d> findPath(Rotation2d startTheta1, Rotation2d  startTheta2,
                                      Rotation2d goalTheta1, Rotation2d goalTheta2) {
        PriorityQueue<Node> openSet = new PriorityQueue<>();
        Map<String, Node> allNodes = new HashMap<>();

        // Initialize start node
        Node start = new Node(startTheta1, startTheta2);
        start.gScore = 0;
        start.fScore = heuristic(start, goalTheta1, goalTheta2);
        openSet.add(start);
        allNodes.put(nodeKey(startTheta1, startTheta2), start);

        while (!openSet.isEmpty()) {
            Node current = openSet.poll();

            if (atGoal(current, goalTheta1, goalTheta2)) {
                return reconstructPath(current);
            }

            // Generate neighbors
            for (int i = -1; i <= 1; i++) {
                for (int j = -1; j <= 1; j++) {
                    if (i == 0 && j == 0) continue;

                    Rotation2d newTheta1 = new Rotation2d(current.theta1.getRadians() + i * error);
                    Rotation2d newTheta2 = new Rotation2d(current.theta2.getRadians() + j * error);
                    String key = nodeKey(newTheta1, newTheta2);

                    if (!configSpace.isValidConfiguration(newTheta1, newTheta2)) continue;

                    Node neighbor = allNodes.getOrDefault(key, new Node(newTheta1, newTheta2));
                    double tentativeGScore = current.gScore + distance(current, neighbor);

                    if (tentativeGScore < neighbor.gScore) {
                        neighbor.parent = current;
                        neighbor.gScore = tentativeGScore;
                        neighbor.fScore = tentativeGScore + heuristic(neighbor, goalTheta1, goalTheta2);
                        
                        if (!openSet.contains(neighbor)) {
                            openSet.add(neighbor);
                        }
                        allNodes.put(key, neighbor);
                    }
                }
            }
        }
        return Collections.emptyList(); // No path
    }

    private List<Translation2d> reconstructPath(Node endNode) {
        LinkedList<Translation2d> path = new LinkedList<>();
        Node current = endNode;
        
        while (current != null) {
            path.addFirst(new Translation2d(current.theta1.getRadians(), current.theta2.getRadians()));
            current = current.parent;
        }
        
        return smoothPath(path);
    }

    private List<Translation2d> smoothPath(List<Translation2d> roughPath) {
        if (roughPath.size() < 3) return roughPath;
        
        List<Translation2d> smoothed = new ArrayList<>();
        smoothed.add(roughPath.get(0));
        
        // Average thetas then add to path
        for (int i = 1; i < roughPath.size()-1; i++) {
            Rotation2d theta1 = new Rotation2d((roughPath.get(i-1).getX() + 
                           roughPath.get(i).getX() + 
                           roughPath.get(i+1).getX()) / 3);
            Rotation2d theta2 = new Rotation2d((roughPath.get(i-1).getY() + 
                           roughPath.get(i).getY() + 
                           roughPath.get(i+1).getY()) / 3);
            smoothed.add(new Translation2d(theta1.getRadians(), theta2.getRadians()));
        }
        
        smoothed.add(roughPath.get(roughPath.size()-1));
        return smoothed;
    }

    // Helper Functions
    private boolean atGoal(Node node, Rotation2d goalTheta1, Rotation2d goalTheta2) {
        return Math.abs(node.theta1.getRadians() - goalTheta1.getRadians()) < error &&
               Math.abs(node.theta2.getRadians() - goalTheta2.getRadians()) < error;
    }

    private double distance(Node a, Node b) {
        return Math.hypot(b.theta1.minus(a.theta1).getRadians(), b.theta2.minus(a.theta2).getRadians());
    }

    private double heuristic(Node node, Rotation2d goalTheta1, Rotation2d goalTheta2) {
        return distance(node, new Node(goalTheta1, goalTheta2));
    }

    private String nodeKey(Rotation2d theta1, Rotation2d theta2) {
        return String.format("%.3f,%.3f", theta1.getRadians(), theta2.getRadians());
    }
}