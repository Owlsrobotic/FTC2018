package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Color;
import android.graphics.Point;

import com.vuforia.Image;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashSet;

public class BlobDetector {
    double[][] binaryImage;
    Bitmap scaledBitmap;

    public BlobDetector(Image image, double threshold, double scaleRatio) {
        Bitmap bitmap = Bitmap.createBitmap(image.getWidth(), image.getHeight(), Bitmap.Config.RGB_565);
        bitmap.copyPixelsFromBuffer(image.getPixels());

        int scaledWidth = (int) (image.getWidth() * scaleRatio);
        int scaledHeight = (int) (image.getHeight() * scaleRatio);
        scaledBitmap = Bitmap.createScaledBitmap(bitmap, scaledWidth, scaledHeight, true);

//      Make a luminance map
        double[][] luminanceMap = new double[scaledWidth][scaledHeight];
        for (int x = 0; x < scaledWidth; x++) {
            for (int y = 0; y < scaledHeight; y++) {
                int color = scaledBitmap.getPixel(x, y);
                final float[] hslValue = {0F, 0F, 0F};
                Color.colorToHSV(color, hslValue);
                luminanceMap[x][y] = (hslValue[2] > threshold) ? 1.0 : 0.0;
            }
        }

        binaryImage = luminanceMap;
    }

    public void renderImage(String path, int minVotes, int minRadius, int maxRadius) {
        ArrayList<int[]> circles = findCircles(minVotes, minRadius, maxRadius);
        int width = binaryImage.length;
        int height = binaryImage[0].length;

        Bitmap bitmap = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888);
        //Render Image
        for (int x = 0; x < width; x++) {
            for (int y = 0; y < height; y++) {
                int color = (binaryImage[x][y] == 1.0) ? Color.WHITE : Color.BLACK;
                bitmap.setPixel(x, y, color);
            }
        }
        //Render Circles
        for (int[] circle : circles) {
            HashSet<Point> points = generateCirclePoints(circle[0], circle[1], circle[2]);
            for (Point point : points) {
                if (point.x >= 0 && point.x < width && point.y >= 0 && point.y < height) {
                    bitmap.setPixel(point.x, point.y, Color.RED);
                }
            }
        }
        File file = new File(path);
        try (FileOutputStream out = new FileOutputStream(file)) {
            bitmap.compress(Bitmap.CompressFormat.PNG, 100, out); // bmp is your Bitmap instance
            System.out.println("Printed to: " + file.getAbsolutePath());
            // PNG is a lossless format, the compression factor (100) is ignored
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public ArrayList<int[]> findCircles(int minVotes, int minRadius, int maxRadius) {
        int imageWidth = binaryImage.length;
        int imageHeight = binaryImage[0].length;

        double[][] edgeDetectImage;
        //Convolve to detect edges
        System.out.println("Edge Detection...");
        edgeDetectImage = new double[imageWidth][imageHeight];
        int n = 3;
        for (int x = 0; x < imageWidth; x++) {
            for (int y = 0; y < imageHeight; y++) {
                //Set pixel to the maximum value difference in nxn box centered at (x, y)
                edgeDetectImage[x][y] = getMaxDifference(x, y, n, binaryImage);
            }
        }
        //Do Hough Circle Transform
        System.out.println("Hough Circle Transform...");
        final int[][][] accumulator = new int[imageWidth][imageHeight][maxRadius];
        //Set initial array
        for (int x = 0; x < imageWidth; x++) {
            for (int y = 0; y < imageHeight; y++) {
                for (int r = 0; r < maxRadius; r++) {
                    accumulator[x][y][r] = 0;
                }
            }
        }


        for (int x = 0; x < imageWidth; x++) {
            for (int y = 0; y < imageHeight; y++) {
                //Find edge points
                if (edgeDetectImage[x][y] == 1.0) {
                    //iterate over all radii
                    for (int r = 1; r < maxRadius; r++) {
                        HashSet<Point> points = generateCirclePoints(x, y, r);
                        for (Point point: points) {
                            int circleX = point.x;
                            int circleY = point.y;
                            if (circleX >= 0 && circleX < imageWidth && circleY >= 0 && circleY < imageHeight) {
                                accumulator[circleX][circleY][r] += 1;
                            }
                        }
                    }
                }
            }
        }

        System.out.println("Extracting Points...");
        ArrayList<int[]> sortedByVote = flatten(accumulator);
        System.out.println("\t Sorting by vote...");
        Collections.sort(sortedByVote, new Comparator<int[]>() {
            @Override
            public int compare(int[] t1, int[] t2) {
                int t1Value = accumulator[t1[0]][t1[1]][t1[2]];
                int t2Value = accumulator[t2[0]][t2[1]][t2[2]];
                if (t1Value < t2Value) {
                    return -1;
                }
                if (t1Value > t2Value) {
                    return 1;
                }
                return 0;
            }
        });

        ArrayList<int[]> candidates = new ArrayList<>();
        for (int i = 0; i < sortedByVote.size(); i++) {
            //Pop off stack sorted list to get value w/ the highest number of votes
            int[] max = sortedByVote.remove(sortedByVote.size() - 1);
            if (accumulator[max[0]][max[1]][max[2]] < minVotes) {
                break;
            }

            boolean shouldSkip = false;
            for (int[] candidate : candidates) {
                //Don't add circles that are in area of another circle
                if (Math.pow(Math.pow(candidate[0] - max[0] , 2) + Math.pow(candidate[1] - max[1] , 2), 0.5) < candidate[2]) {
                    shouldSkip = true;
                    break;
                }

                if (candidate[2] < minRadius) {
                    shouldSkip = true;
                    break;
                }
            }
            if (!shouldSkip) {
                System.out.println("x center: " + max[0]);
                System.out.println("y center: " + max[1]);
                System.out.println("radius: " + max[2]);
                System.out.println("votes: " + accumulator[max[0]][max[1]][max[2]]);
                System.out.println("--------------------------------");
                candidates.add(max);
            }

        }

        return candidates;
    }


    public ArrayList<int[]> flatten(int[][][] accumulator) {
        System.out.println("SIZE: " + accumulator.length * accumulator[0].length * accumulator[0][0].length);
        ArrayList<int[]> result = new ArrayList<>();
        for (int x = 0; x < accumulator.length; x++) {
            for (int y = 0; y < accumulator[0].length; y++) {
                for (int r = 0; r < accumulator[0][0].length; r++) {
                    result.add(new int[]{x, y, r});
                }
            }
        }

        return result;
    }

    public HashSet<Point> generateCirclePoints(int x, int y, int r) {
        HashSet<Point> points = new HashSet<>();
        for (int i = 0; i <= 90; i+= 1) {
            int dX = (int) (r * Math.cos(Math.toRadians(i)));
            int dY = (int) (r * Math.sin(Math.toRadians(i)));
            points.add(new Point(x + dX, y + dY));
            points.add(new Point(x + dX, y - dY));
            points.add(new Point(x - dX, y + dY));
            points.add(new Point(x - dX, y - dY));
        }

        return points;
    }

    public double getMaxDifference(int x, int y, int n, double[][] image) {
        double currentValue = image[x][y];
        int width = image.length;
        int height = image[0].length;

        //Create bounding box of size nxn centered around (x, y)
        int minX = Math.max(0, x - (n/2));
        int maxX = Math.min(width, x + (n/2));
        int minY = Math.max(0, y - (n/2));
        int maxY = Math.min(height, y + (n/2));

        double maxDifference = 0.0;
        for (int posX = minX; posX < maxX; posX++) {
            for (int posY = minY; posY < maxY; posY++) {
                if (Math.abs(currentValue - image[posX][posY]) > maxDifference) {
                    maxDifference = Math.abs(currentValue - image[posX][posY]);
                }
            }
        }

        return maxDifference;
    }
}
