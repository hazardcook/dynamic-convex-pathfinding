package com.hazardcook.dynamic_convex_pathfinding.test;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.awt.geom.Line2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import javax.swing.JFrame;
import javax.swing.JPanel;

import org.junit.Test;

import com.hazardcook.dynamic_convex_pathfinding.BodyWrapper;
import com.hazardcook.dynamic_convex_pathfinding.Path;
import com.hazardcook.dynamic_convex_pathfinding.TitusPathFinder;
import com.hazardcook.dynamic_convex_pathfinding.Vec2;
import com.hazardcook.dynamic_convex_pathfinding.WorldWrapper;

public class TestingPathFinding {
	
	/**
	 * Implementation of a {@link BodyWrapper} that simply contains a rectangle
	 * @author nathan titus
	 * @version 1.0
	 *
	 */
	public class BodyWrapperTestClass implements BodyWrapper {
		public Rectangle2D r;
	}

	/**
	 * Implementation of a {@link WorldWrapper} that contains an array of {@link BodyWrapperTestClass}es
	 * Does raycasts by intersecting lines with rectangles in the body wrapper implementation, does
	 * detection by checking if those rectangles contain the given point
	 * @author nathan titus
	 * @version 1.0
	 *
	 */
	private class WorldWrapperTestClass implements WorldWrapper {

		ArrayList<BodyWrapperTestClass> rectangles = new ArrayList<BodyWrapperTestClass>();
		
		@Override
		public List<BodyWrapper> raycast(Vec2 start, Vec2 end) {
			List<BodyWrapper> bodies = new ArrayList<BodyWrapper>();
			for(int i = 0; i < rectangles.size(); i++){
				BodyWrapperTestClass body = rectangles.get(i);
				Line2D.Double line = new Line2D.Double(start.x, start.y, end.x, end.y);
				if(body.r.intersectsLine(line) || line.intersects(body.r)){
					bodies.add(body);
				}
			}
			return bodies;
		}

		@Override
		public boolean raycast(Vec2 start, Vec2 end, BodyWrapper body) {
			if(body instanceof BodyWrapperTestClass){
				BodyWrapperTestClass b = (BodyWrapperTestClass) body;
				Line2D.Double line = new Line2D.Double(start.x, start.y, end.x, end.y);
				if(b.r.intersectsLine(line) || line.intersects(b.r)){
					return true;
				}
			}
			return false;
		}

		@Override
		public boolean detect(Vec2 point) {
			for(BodyWrapperTestClass body:rectangles){
				if(body.r.contains(point.x, point.y)){
					return true;
				}
			}
			return false;
		}
		
	}
	
	JFrame frame = new JFrame("Test");
	
	@Test
	public void testWithOneRectangle() {
		/*
		 * Setup portion
		 */
		Vec2 start = new Vec2(0,0);
		Vec2 end = new Vec2(10,0);
		WorldWrapperTestClass world = new WorldWrapperTestClass();
		BodyWrapperTestClass body = new BodyWrapperTestClass();
		body.r = new Rectangle2D.Double(4.5, -0.5, 1.0, 1.0);
		world.rectangles.add(body);
		
		/*
		 * Pathfinding portion
		 */
		TitusPathFinder pFinder = new TitusPathFinder();
		Path path = pFinder.shortestPath(start, end, world);
		assert(path.locations.size() == 3);
	}
	
	@Test
	public void testWith100Rectangles(){
		/*
		 * Setup portion
		 */
		Vec2 start = new Vec2(0,0);
		Vec2 end = new Vec2(100,0);
		WorldWrapperTestClass world = new WorldWrapperTestClass();
		Random random = new Random();
//		random.setSeed(42);
		for(int i = 0; i < 100; i++){
			BodyWrapperTestClass body = new BodyWrapperTestClass();
			body.r = new Rectangle2D.Double(random.nextDouble()*90, -50.0 + random.nextDouble()*90, random.nextDouble()*10, random.nextDouble()*10);
			if(body.r.contains(start.x, start.y) || body.r.contains(end.x, end.y)){
				i -= 1;
				continue;
			}
			world.rectangles.add(body);
		}
		
		/*
		 * Pathfinding portion
		 */
		TitusPathFinder pFinder = new TitusPathFinder();
		pFinder.initIterPath(start, end, world);
		Path path = pFinder.iterShortestPath();
		final PathCarrier pc = new PathCarrier(path);
		/*
		 * Render portion
		 */
		frame.setSize(800, 800);
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		JPanel panel = new JPanel(){
			private static final long serialVersionUID = 1L;
			
			@Override
			public void paintComponent(Graphics g){
				g.setColor(Color.WHITE);
				g.fillRect(0, 0, 800, 800);
				g.setColor(Color.BLACK);
				for(int i = 0; i < world.rectangles.size(); i++){
					Rectangle2D r = ((BodyWrapperTestClass) world.rectangles.get(i)).r;
					g.drawRect((int)r.getX()*8, (int)r.getY()*8+400, (int)r.getWidth()*8, (int)r.getHeight()*8);
				}
				g.setColor(Color.RED);
				for(int i = 0; i < pc.path.locations.size() - 1; i++){
					Vec2 s = pc.path.locations.get(i);
					Vec2 e = pc.path.locations.get(i + 1);
					g.drawLine((int)s.x*8, (int)(s.y*8)+400, (int)e.x*8, (int)(e.y*8)+400);
				}
			}
		};
		panel.setSize(800, 800);
		frame.add(panel);
		frame.setVisible(true);
		KeyListener kListener = new KeyListener(){
			@Override
			public void keyTyped(KeyEvent e) {
				// TODO Auto-generated method stub
			}
			@Override
			public void keyPressed(KeyEvent e) {
				int key = e.getKeyCode();
				if(key == KeyEvent.VK_ENTER){
					pc.path = pFinder.iterShortestPath();
					panel.repaint();
				}
				if(key == KeyEvent.VK_F){
					pc.path = pFinder.shortestPath(start, end, world);
					panel.repaint();
				}
				if(key == KeyEvent.VK_SPACE){
					world.rectangles.clear();
					for(int i = 0; i < 100; i++){
						BodyWrapperTestClass body = new BodyWrapperTestClass();
						body.r = new Rectangle2D.Double(random.nextDouble()*90, -50.0 + random.nextDouble()*90, random.nextDouble()*10, random.nextDouble()*10);
						if(body.r.contains(start.x, start.y) || body.r.contains(end.x, end.y)){
							i -= 1;
							continue;
						}
						world.rectangles.add(body);
					}
					panel.repaint();
				}
				System.out.println("key pressed");
			}
			@Override
			public void keyReleased(KeyEvent e) {
				// TODO Auto-generated method stub
				
			}
		};
		frame.addKeyListener(kListener);
	
		while(true);	//	execute until window closed
	}
	
	private class PathCarrier {
		Path path;
		PathCarrier(Path path){
			this.path = path;
		}
	}
	

}
