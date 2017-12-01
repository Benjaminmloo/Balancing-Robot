import java.net.*;
import java.util.Arrays;
import java.util.Scanner;



public class Client {

	private final static int PACKETSIZE = 100 ;
	private DatagramSocket socket;
	private DatagramPacket sendPacket;
	private DatagramPacket receivePacket;
	
	private boolean quiteMode = false;
	
	private InetAddress serverIP;
	private int serverPort;
	
	private static Scanner sc;
	
	
	public Client(String ipStr, String portStr)
	{
		try {
			socket = new DatagramSocket() ;
			serverIP = InetAddress.getByName( ipStr );
			serverPort = Integer.parseInt( portStr );
		} catch( Exception e ) {
			System.out.println( e ) ;
		}
	}
	
	
	public static void main( String args[] )
	{

	    System.out.println("Welcome to use the Client\n");
	    
	    /* init ip and port */
		String ipStr;
		String portStr;
	      if( args.length != 2 )
	      {
	    	  ipStr = "127.0.0.1";
	    	  portStr = "959";
	      }
	      else {
	    	  ipStr = args[0];
	    	  portStr = args[1];
	      }
	      
	      
	      sc = new Scanner(System.in);	      
	      Client client = new Client(ipStr, portStr);

	      //client.remoteCall(new String[] {"moveFoward","1"});
	      while (true)
	      {
	    	  System.out.println("Getting new user input\n");
		      String message = getUserInputString();
	    	  System.out.println("Calling remote device\n");
		      client.remoteCall(message);
	      }
    }
	
	private static String getUserInputString() {

		String result = "";
		while (result.length() < 1) {
			result = sc.nextLine();
		} 
		return result;
	}
	
	


	
	public void remoteCall(String str) {
		if (str == null || str.length() < 1) return; //pre-condition
		
		try {
	   		 byte[] sendData = str.getBytes();
	   		 sendPacket = new DatagramPacket( sendData, sendData.length, serverIP, serverPort ) ;
	   		 socket.send( sendPacket );
	       		 
	   		 receivePacket = new DatagramPacket( new byte[PACKETSIZE], PACKETSIZE ) ;
	   		 socket.setSoTimeout(1000);
		     socket.receive( receivePacket ) ;
		     byte[] receivedData = Arrays.copyOfRange(receivePacket.getData(),0,receivePacket.getLength());
	         println( "Client received: " + receivePacket.getAddress() + " " + receivePacket.getPort() + ": \"" + new String(receivedData) +"\"") ;
		       	        		 
	     }
	     catch( Exception e )
	     {
	        System.out.println( e ) ;
	     }
	}
	
	public void remoteCall(String args[]) {
		if (args == null || args.length < 1) return; //pre-condition
		try {
	   		 byte[] sendData = formSendData(args);
	   		 sendPacket = new DatagramPacket( sendData, sendData.length, serverIP, serverPort ) ;
	   		 socket.send( sendPacket );
	   		 
	   		 
	   		 receivePacket = new DatagramPacket( new byte[PACKETSIZE], PACKETSIZE ) ;
		     socket.receive( receivePacket ) ;
		     byte[] receivedData = Arrays.copyOfRange(receivePacket.getData(),0,receivePacket.getLength());
	         println( "Client received: " + receivePacket.getAddress() + " " + receivePacket.getPort() + ": \"" + new String(receivedData) +"\"") ;
		         
		     //checkOpcode(expectedOpcode, getOpcode());		        		 
	     }
	     catch( Exception e )
	     {
	        System.out.println( e ) ;
	     }
	}
	
	
	public byte[] formSendData(String args[])
	{
		String result = "01"+" "+ "" +" ";
		for (String s : args) {
			result = result + s + " ";
		}
		return result.getBytes();
	}
	
	public byte[] formSendData(String str)
	{
		String result = "01 ";
		return (result + str + " ").getBytes();
	}
	
/*
	public void testCase1()
	{
	      try
	      {
		         		String message = "0101dataString";
		         		String expectedOpcode = "02";
		         		
		    			byte [] start = message.getBytes();
		    			byte [] end = {0};		         		
		        		 byte [] data = concateByteArray(start, end);
		        		 
		        		 
		        		 int serverPort = 777;
		        		 InetAddress host;
		        		 
		        		 byte[] sendData = formSendData("MoveForward");
		        		 sendPacket = new DatagramPacket( sendData, sendData.length, serverIP, serverPort ) ;
		        		 socket.send( sendPacket ) ;
		        		 
		        		 
		        		 println( "Client sending");

		        		 println( "Client sent");
		        		 		 
		        		 receivePacket = new DatagramPacket( new byte[PACKETSIZE], PACKETSIZE ) ;
		        		 println( "Client receiving");
		        		 socket.receive( receivePacket ) ;
		        		 println( "Client received: " + receivePacket.getAddress() + " " + receivePacket.getPort() + ": " + new String(receivePacket.getData()).trim() ) ;
		        		 
		        		 System.out.print("test case 2: valid data '0101dataString' - ");
		        		 checkOpcode(expectedOpcode, getOpcode());		        		 
	     }
	     catch( Exception e )
	     {
	        System.out.println( e ) ;
	     }
	}
	*/

	
	
    /*
    while(true){
    		String message = "Hello";
   		 byte [] data = message.getBytes();
   		 sendPacket = new DatagramPacket( data, data.length, host, port ) ;
   		 println( "Client sending");
   		 socket.send( sendPacket ) ;
   		 println( "Client sent");
   		 		        		 
   		 receivePacket = new DatagramPacket( new byte[PACKETSIZE], PACKETSIZE ) ;
   		 println( "Client receiving");
   		 socket.receive( receivePacket ) ;
   		 println( "Client received: " + receivePacket.getAddress() + " " + receivePacket.getPort() + ": " + new String(receivePacket.getData()).trim() ) ;
    }
    */
	public void checkOpcode(String expected, String actual){
		print( "expected: "+ expected) ;
		print( "actual: " +actual) ;
		 if (actual.equals(expected)) System.out.println(" - pass");
		 else System.out.println(" - fail");
	}
	
	public byte[] concateByteArray(byte[] a, byte[] b){
		byte[] c = new byte[a.length + b.length];
		System.arraycopy(a, 0, c, 0, a.length);
		System.arraycopy(b, 0, c, a.length, b.length);
		return c;
	}


	/*
	public static void testCase2()
	{
	      try
	      {
		         		String message = "dataString";
		         		String expectedOpcode = "03";
		         		
		    			byte [] start = message.getBytes();
		    			byte [] end = {0};		         		
		        		 byte [] data = concateByteArray(start, end);
		         		
		        		 //byte [] data = message.getBytes();
		        		 sendPacket = new DatagramPacket( data, data.length, serverIP, serverPort ) ;
		        		 println( "Client sending");
		        		 socket.send( sendPacket ) ;
		        		 println( "Client sent");
		        		 		 
		        		 receivePacket = new DatagramPacket( new byte[PACKETSIZE], PACKETSIZE ) ;
		        		 println( "Client receiving");
		        		 socket.receive( receivePacket ) ;
		        		 println( "Client received: " + receivePacket.getAddress() + " " + receivePacket.getPort() + ": " + new String(receivePacket.getData()).trim() ) ;
		        		 println( "Client received: " + new String(receivePacket.getData()).trim() ) ;
		        		 
		        		 System.out.print("test case 1: invalid data 'dataString' - ");
		        		 checkOpcode(expectedOpcode, getOpcode());		        		 
	     }
	     catch( Exception e )
	     {
	        System.out.println( e ) ;
	     }
	}
	
	
	public static void testCase3()
	{
	      try
	      {
		         		String message = "0177dataString";
		         		String expectedOpcode = "03";
		         		
		    			byte [] data = message.getBytes();
		         		
		        		 sendPacket = new DatagramPacket( data, data.length, serverIP, serverPort ) ;
		        		 println( "Client sending");
		        		 socket.send( sendPacket ) ;
		        		 println( "Client sent");
		        		 		 
		        		 receivePacket = new DatagramPacket( new byte[PACKETSIZE], PACKETSIZE ) ;
		        		 println( "Client receiving");
		        		 socket.receive( receivePacket ) ;
		        		 println( "Client received: " + receivePacket.getAddress() + " " + receivePacket.getPort() + ": " + new String(receivePacket.getData()).trim() ) ;
		        		 
		        		 System.out.print("test case 3: missing ending 0 '0177dataString' - ");
		        		 checkOpcode(expectedOpcode, getOpcode());		        		 
	     }
	     catch( Exception e )
	     {
	        System.out.println( e ) ;
	     }
	}
	
	public static void testCase4()
	{
	      try
	      {
		         		String message = "9977dataString";
		         		String expectedOpcode = "03";
		         		
		    			byte [] start = message.getBytes();
		    			byte [] end = {0};		         		
		        		 byte [] data = concateByteArray(start, end);
		         		
		        		 //byte [] data = message.getBytes();
		        		 sendPacket = new DatagramPacket( data, data.length, serverIP, serverPort ) ;
		        		 println( "Client sending");
		        		 socket.send( sendPacket ) ;
		        		 println( "Client sent");
		        		 		 
		        		 receivePacket = new DatagramPacket( new byte[PACKETSIZE], PACKETSIZE ) ;
		        		 println( "Client receiving");
		        		 socket.receive( receivePacket ) ;
		        		 println( "Client received: " + receivePacket.getAddress() + " " + receivePacket.getPort() + ": " + new String(receivePacket.getData()).trim() ) ;
		        		 
		        		 System.out.print("test case 4: invalid OpCode '9977dataString' - ");
		        		 checkOpcode(expectedOpcode, getOpcode());		        		 
	     }
	     catch( Exception e )
	     {
	        System.out.println( e ) ;
	     }
	}
	
	public static void testCase5()
	{
	      try
	      {
		         		String message = "01XXdataString";
		         		String expectedOpcode = "03";
		         		
		    			byte [] start = message.getBytes();
		    			byte [] end = {0};		         		
		        		 byte [] data = concateByteArray(start, end);
		         		
		        		 //byte [] data = message.getBytes();
		        		 sendPacket = new DatagramPacket( data, data.length, serverIP, serverPort ) ;
		        		 println( "Client sending");
		        		 socket.send( sendPacket ) ;
		        		 println( "Client sent");
		        		 		 
		        		 receivePacket = new DatagramPacket( new byte[PACKETSIZE], PACKETSIZE ) ;
		        		 println( "Client receiving");
		        		 socket.receive( receivePacket ) ;
		        		 println( "Client received: " + receivePacket.getAddress() + " " + receivePacket.getPort() + ": " + new String(receivePacket.getData()).trim() ) ;
		        		 
		        		 System.out.print("test case 5: invalid block number '01XXdataString' - ");
		        		 checkOpcode(expectedOpcode, getOpcode());		        		 
	     }
	     catch( Exception e )
	     {
	        System.out.println( e ) ;
	     }
	}
	*/

	
	/*
	public static String getOpcode(){
    	return new String(new byte[] {receivePacket.getData()[0], receivePacket.getData()[1]});
	}
	
	public int getBlockNum(){
		//invalid opcode
    	if (receivePacket.getData()[0] != (byte) 0) return -1;
    	//check block number
    	if (receivePacket.getData()[1] == (byte)3 || receivePacket.getData()[1] == (byte)4) {
	    	return ((int) Math.pow(2, 8))*unsignedToBytes(receivePacket.getData()[2]) + unsignedToBytes(receivePacket.getData()[3]);
    	}
    	return -1;
	}
	
	public byte[] intToByte(int value) {
	    return new byte[] {
	            (byte)(value >>> 8),
	            (byte)value};
	}
	
	  public int unsignedToBytes(byte b) {
		    return b & 0xFF;
		  }
	  
	
	
	public static void stop()
	{
  	  println ("Client closing down");
      if( socket != null ) socket.close() ;
    }
	*/
	public void print(String str)
	{
  	  if (!quiteMode) System.out.print (str);
    }
	
	public void println(String str)
	{
  	  if (!quiteMode) System.out.println (str);
    }
    
	
}
