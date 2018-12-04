import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.*;
class TwoEnds{
	public CubeNode end1;
	public CubeNode end2;	
	public TwoEnds()
	{
		end1=null;
		end2=null;
	}
	public TwoEnds(CubeNode end11,CubeNode end22)
	{
		end1=end11;
		end2=end22;
	}
}

public class BD_IDAStar {

	public static final int[] corners = readHeuristics(88179840, "corners.csv");
	public static final int[] edgesSetOne = readHeuristics(42577920, "edgesSetOne.csv");
	public static final int[] edgesSetTwo = readHeuristics(42577920, "edgesSetTwo.csv");
	public static int nextBound;
	public static int targetBound;
	public static int nodesVisited;
	public static PriorityQueue<CubeNode> frontier = new PriorityQueue<CubeNode>();
	public static HashSet<CubeNode> explored = new HashSet<CubeNode>();
	public static PriorityQueue<CubeNode> frontier_end = new PriorityQueue<CubeNode>();
	public static HashSet<CubeNode> explored_end = new HashSet<CubeNode>();
	public static HashSet<String> path_start = new HashSet<String>();
	public static HashSet<String> path_end = new HashSet<String>();

	/**
	 * Performs the BD_IDA* search for our Rubik's cube.
	 * @param startState the starting state of the cube
	 * @param verbose true if we want to print out more details about the BD_IDA* algorithm
	 * @return the string that represents the optimal solution
	 */
	public static String performBD_IDAStar(char[] startState, boolean verbose) {
		// Don't bother wasting CPU cycles for an already solved Cube
		if (Arrays.equals(startState, Cube.GOAL.toCharArray())) {
			return "The given cube is already in a solved state";
		}
		// Initialize the root node with the start state
		CubeNode start = new CubeNode(startState, corners[Integer.parseInt(Cube.encodeCorners(startState))]);
		// Initialize the end node with the goal state
		CubeNode target = new CubeNode(Cube.GOAL.toCharArray(), corners[Integer.parseInt(Cube.encodeCorners(Cube.GOAL.toCharArray()))]);
		// And put the start node on the openSet
		if (verbose) {
			System.out.println("Heuristic value of the start state: " + start.heuristic);
			System.out.println("Heuristic value of the goal state: " + target.heuristic);
		}
		// Initialize nextBound with our starting heuristic value
		nextBound = start.heuristic;
		// Initialize endBound with our end heuristic value
		targetBound = target.heuristic;
		// Initialize nodesVisited
		nodesVisited = 0;
		// The end node once Bidirectional IDA* finishes
		TwoEnds solution=new TwoEnds();
		while(solution.end1==null && solution.end2==null)
		{
		// Adding the Start node in the Priority Queue	
		frontier.add(start);
		path_start.add(new String(startState));
		// Adding the End node in the Priority Queue
		frontier_end.add(target);
		path_end.add(new String(Cube.GOAL));
		//searching for the solution where the Top-Down and Bottom-Up search meets
		solution= search(nextBound,targetBound,verbose);
		//System.out.println(solution);
		//per the Iterative Deepening approach clearing the Queues if the solution is not found at particular bound.
		frontier.clear();
		explored.clear();
		frontier_end.clear();
		explored_end.clear();
		path_start.clear();
		path_end.clear();
		//Increasing the bounds for both the searches.
		nextBound++;
		targetBound++;
		}
		if (verbose) {
			System.out.println("Solved!");
			System.out.println("Total # of nodes visited: " + nodesVisited);
		}

		
		// There is possibility that top approach may find solution with one step and the bottom-up approach queue may remain empty if both are non empty
		if(solution.end2!=null)
		{
		String endp=solution.end2.path;
		String reverse="";
		for(int i = endp.length() - 1; i >= 0; i--)
        {
            reverse = reverse + endp.charAt(i);
        }
        char[] gfg = reverse.toCharArray();
        for(int i = 0; i < reverse.length() - 1; i++)
        {
        	char t=gfg[i];
        	gfg[i]=gfg[i+1];
        	gfg[i+1]=t;
        	i=i+1;
        }
        String endk=new String(gfg);
        String edit_first=formatOptimalSolution(solution.end1.path);
        String edit_second=formatOptimalSolution(endk);
        char[] format = edit_second.toCharArray();
        for(int i = 0; i < edit_second.length() - 1; i++)
        {
        	int t=Integer.parseInt(Character.toString(format[i+1]));
        	format[i]=format[i];
        	if(t==1)
        		format[i+1]=Integer.toString(3).charAt(0);
        	else if(t==3)
        		format[i+1]=Integer.toString(1).charAt(0);
        	else
        		format[i+1]=Integer.toString(t).charAt(0);
        	i=i+1;
        }
        String adjust_second=new String(format);
        if(verbose)
		{
			if(solution.end1!=null)
				System.out.println("Top-down search path received as "+solution.end1.path+" and formatted to "+edit_first);
			System.out.println("Bottom-up search path received as "+solution.end2.path+" and formatted to "+adjust_second);
		}

        return formatOptimalSolution(edit_first+adjust_second);
    	}
        //If only the top approach finds the solution.
		return formatOptimalSolution(solution.end1.path);
	}

	/**
	 * The recursive expanding function that will expand nodes as per
	 * the rules of BD_IDA*
	 * @param bound the current bound - used to determine if we should
	 *              expand nodes or not
	 * @return the node representation of the goal state
	 */
	private static TwoEnds search(int bound,int ultabound,boolean verbose) {
		TwoEnds end=new TwoEnds();
		nodesVisited++;
		CubeNode current=null;
		CubeNode current_end=null;
		//System.out.println("Top-down bound "+bound+" Bottom-up "+ultabound);

		while (!frontier.isEmpty() || !frontier_end.isEmpty()) {
			nodesVisited++;
			if(!frontier.isEmpty())
			{	
				
				//Considers the first node in queue for top-down approach
				current	=frontier.poll();
				//System.out.println("Top Down approach current path: "+current.path+" state: "+new String(current.state));
				if(!explored.contains(current))
				{
					//Adds it into the explored nodes list;
					explored.add(current);
					//Stores only the State of the cube;
					path_start.add(new String(current.state));
					//checks if the Goal state is reached or the bottom up approach has any of the traversed state;
					if (path_end.contains(new String(current.state)))
					{
						
						//Finds out the exact nodes which matched from the bottom up approach;
						
					
							for(CubeNode t : explored_end)
							{
								String S1=new String(current.state);
								String S2=new String(t.state);
								if(S1.equals(S2))
							{
								if (verbose)
								{
								System.out.println("Match found");
								System.out.println("current Top-down path: "+current.path+" state: "+new String(current.state));
								System.out.println("current Bottom-up path: "+t.path+" state: "+new String(t.state));
								}

								end.end1=current;
								end.end2=t;
								return end;
							}
							}
	
						

					}
					ArrayList<CubeNode> successors = CubeNode.getSuccessors(current);

					for(CubeNode successor: successors)
					{
						//String s=new String(successor.state);
						//System.out.println(successor.path+"  "+s+" "+Cube.GOAL);
						//if(s.equals(Cube.GOAL))
						
						if(!explored.contains(successor))
						{
						
							//Creating heuristic and compare with the bound to restrict the state space
							int f = current.g + successor.heuristic;
							successor.g = current.g + 1;
							if (f <= bound && !explored.contains(successor)) {
							frontier.add(successor);
							}
						}

					}

				}
			}
			//Applying the same approach to bottom up search
			if(!frontier_end.isEmpty())
			{
				nodesVisited++;
				current_end	=frontier_end.poll();
				//System.out.println("Bottom up approach current path: "+current_end.path+" state: "+new String(current_end.state));
				if(!explored_end.contains(current_end))
				{
					
					explored_end.add(current_end);
					path_end.add(new String(current_end.state));
					if(path_start.contains(new String(current_end.state)))
					{
					
						for(CubeNode t : explored)
							{
								String S1=new String(t.state);
								String S2=new String(current_end.state);
								if(S1.equals(S2))
							{
								if(verbose)
								{
								System.out.println("Match found");
								System.out.println("current Top-down path: "+t.path+" state: "+new String(t.state));
								System.out.println("current Bottom-up path: "+current_end.path+" state: "+new String(current_end.state));
								}
	
								end.end1=t;
								end.end2=current_end;
								return end;
							}
							}



					}
					ArrayList<CubeNode> successors = CubeNode.getSuccessors(current_end);
					for(CubeNode successor: successors)
					{
						//System.out.println(successor.path+" bottom "+new String(successor.state));
						if(!explored_end.contains(successor))
						{
							int f = current_end.g + successor.heuristic;
							successor.g = current.g + 1;
							if (f <= ultabound && !explored_end.contains(successor)) {
							frontier_end.add(successor);
						}
						}

					}

				}
			}
			}
		// We did not find the solution at this bound
		return end;
	}
			
		

	/**
	 * Formats the solution so that it prints out nicely. Without this function,
	 * a solution would duplicate turns of the same face eg: O1R1R1R1.
	 * This function will turn that solution into a prettier, O1R3.
	 * @param solution the unformatted solution
	 * @return a properly formatted optimal solution
	 */
	private static String formatOptimalSolution(String solution) {
		try {
			char[] s = solution.toCharArray();
			// Initialize the solution with the beginning 2 characters
			String optimalSolution = solution.substring(0, 2);
			for (int i = 2; i < s.length; i ++) {
				// Add each character to the optimal solution
				optimalSolution += s[i];
				// If the current character is equal to the last character in the string
				// and if i % 2 == 0 so that we are only comparing characters
				if (s[i] == s[i - 2] && i % 2 == 0) {
					// Get the number that we're going to increment
					Integer oldNumber = Integer.parseInt(optimalSolution.substring(
							optimalSolution.length() - 2, optimalSolution.length() - 1));
					// Trim the optimal solution to remove the old number
					optimalSolution = optimalSolution.substring(0, optimalSolution.length() - 2);
					// Add the incremented value
					optimalSolution += (oldNumber + 1)%4;
					// Manually increment i so that we skip over the values we just handled
					// in this case.
					i++;
				}
			}
			String answer="";
			for(int i=1;i<optimalSolution.length();i++)
			{
				if(optimalSolution.charAt(i)!='0')
				{
						answer+=optimalSolution.charAt(i-1);
						answer+=optimalSolution.charAt(i);
				}
				i++;

			}
			return answer;
		} catch (Exception e) {
			// If anything abnormal happens while trying to format the string,
			// just return the non-pretty version of it. This is a fail-safe.
			return solution;
		}
	}

	/**
	 * Reads the CSV files for the heuristics and returns an int[]
	 * where the values are the heuristic values
	 * @param h the size of the array
	 * @param fileName the name of the CSV file to read from
	 * @return an int[]
	 */
	private static int[] readHeuristics(int h, String fileName) {
		// Our corners heuristics array will have 88179840
		// elements, but not all of them will have a value
		// as we only calculated heuristics for valid corner
		// positions starting at the goal state rather than
		// all possible permutations of corners.
		int[] heuristics = new int[h];
		FileReader file = null;
		String line;
		try {
			file = new FileReader(fileName);
			BufferedReader reader = new BufferedReader(file);
			while ((line = reader.readLine()) != null) {
				// For each line, split by the comma
				String[] lineData = line.split(",");
				// lineData[0] will be the encoded corner value
				// lineData[1] will be the calculated heuristic
				if (!(lineData[0].equals("") || lineData[1].equals(""))) {
					heuristics[Integer.parseInt(lineData[0])] = Integer.parseInt(lineData[1]);
				}
			}
		} catch (FileNotFoundException e) {
			throw new RuntimeException("File not found");
		} catch (IOException e) {
			throw new RuntimeException("IO error occurred");
		} finally {
			if (file != null) {
				try {
					file.close();
				} catch (IOException e) {
					e.printStackTrace();
				}
			}
		}

		return heuristics;
	}

	/**
	 * A quick tester for BD_IDA*
	 * @param args
	 */
	public static void main(String[] args) {
		Cube cube = new Cube("input/cube07");
		//cube = Cube.generateRandomCube();
		System.out.println(cube);
		String result = BD_IDAStar.performBD_IDAStar(cube.state, true);
		System.out.println("Final Solution path "+result);
	}
}
