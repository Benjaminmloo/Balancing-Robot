
import java.io.IOException;
import java.net.*;
import java.util.Arrays;

public class Server {

	private final static int PACKETSIZE = 100 ;
	private static DatagramSocket socket;
	private static DatagramPacket sendPacket;
	private static DatagramPacket receivePacket;
	private static boolean quiteMode = false;
	static String portStr;
	static int blockNum = 0;
	
	public static void main( String args[] )
	{
	      if( args.length != 1 )
	      {
	         println( "server use default port: 959" ) ;
	         portStr  = "959";
	      }
	      else {
	    	  portStr = args[0] ;
	      }
		run();
    }
	
	

	
	public static void run()
	{
	      try
	      {
	    	 int port = Integer.parseInt( portStr ) ;
	         // Construct the socket
	         socket = new DatagramSocket( port ) ;

	         while(true)
	         {
		        println( "Server receiving on port " + port ) ;
	    		receivePacket = new DatagramPacket( new byte[PACKETSIZE], PACKETSIZE ) ;
	    	    socket.receive( receivePacket ) ;
			     byte[] receivedData = Arrays.copyOfRange(receivePacket.getData(),0,receivePacket.getLength());
		         println( "Server received: " + receivePacket.getAddress() + " " + receivePacket.getPort() + ": \"" + new String(receivedData) +"\"") ;
			    
		         /*
		         String[] cmd = new String[]{"/bin/sh", "path/to/script.sh"};
		         //String [] command = {"python C:\\Users\\daweichen\\eclipse-workspace\\udp_v1\\script.py"};//TO DO - try catch
		         Process p = Runtime.getRuntime().exec(cmd);
		         */
		        
		        processReceived(new String(receivedData).trim());
		         
	            
	    		byte [] sendData = chooseReply();
	    		println( "Server reply: " + new String(sendData));
	    		sendPacket = new DatagramPacket( sendData, sendData.length, receivePacket.getAddress(), receivePacket.getPort() ) ;
	    		println( "Server sending");
	    	    socket.send( sendPacket ) ;
	    	    println( "Server sent");
	    	    
	    		blockNum++;
	    		
	        }  
	     }
	     catch( Exception e )
	     {
	        System.out.println( e ) ;
	     }
	      finally
	      {
	    	  stop();
	      }
	}
	
	public static void processReceived (String str)
	{
		if (str.equals("w")) callSerialWrite("w");
		if (str.equals("a")) callSerialWrite("a");
		if (str.equals("d")) callSerialWrite("d");
		if (str.equals("s")) callSerialWrite("s");
		if (str.equals("e")) callSerialWrite("e");
		if (str.equals("f")) callSerialWrite("f");
		if (str.equals("b")) callSerialWrite("b");
		if (str.equals("l")) callSerialWrite("l");
		if (str.equals("r")) callSerialWrite("r");
		
		if (str.equals("+")) callSerialWrite("+");
		if (str.equals("-")) callSerialWrite("-");
	}
	
	public static void callSerialWrite( String id )
	{
		String cmd = "python serial_"+id+".py";

		String[] env = {"PATH=/bin:/usr/bin/"};
		
		try {
			Runtime.getRuntime().exec(cmd, env);
		} catch (IOException e) {
			System.out.println("ERROR");
			e.printStackTrace();
		}
	}
	
	/* decode the data and choose correct reply message*/
	/* reply "0266" if data is valid */
	/* reply "0377Invalid OpCode"+'0' if data is invalid */
	public static byte[] chooseReply(){
		
		return "02".getBytes();
		/*
		//String ending = new String(new byte[] {receivePacket.getData()[receivePacket.getData().length -1]});
		//println(ending);
		if (getOpcode().equals("01") && (receivePacket.getData()[receivePacket.getLength() -1] == 0))
		{
			String ACKstr = "0266";
			byte [] ACK = ACKstr.getBytes();
			return ACK;

		}
		else  //error case, wrong opcode or ending not zero byte
		{
			String startStr = "0377";
			byte [] start = startStr.getBytes();
			String messageStr = "Invalid OpCode";
			byte [] message = messageStr.getBytes();
			byte [] end = {0};
			return concateByteArray(concateByteArray(start, message), end);
		}
		*/
	}
	
	public static byte[] concateByteArray(byte[] a, byte[] b){
		byte[] c = new byte[a.length + b.length];
		System.arraycopy(a, 0, c, 0, a.length);
		System.arraycopy(b, 0, c, a.length, b.length);
		return c;
	}

	
	
	public static String getOpcode(){
    	return new String(new byte[] {receivePacket.getData()[0], receivePacket.getData()[1]});
	}

	
	public static void stop()
	{
  	  println ("Client closing down");
      if( socket != null ) socket.close() ;
    }
	
	public static void print(String str)
	{
  	  if (!quiteMode) System.out.print (str);
    }
	
	public static void println(String str)
	{
  	  if (!quiteMode) System.out.println (str);
    }
	
}
