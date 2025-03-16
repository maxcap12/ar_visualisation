#!/usr/bin/env python3
import asyncio
import websockets
import json
import signal
import sys

# Configuration
SERVER_HOST = "172.25.65.93"  # Update this to your ROS server's IP
SERVER_PORT = 8765        # Update this to your ROS server's port

async def test_connection():
    uri = f"ws://{SERVER_HOST}:{SERVER_PORT}"
    print(f"Attempting to connect to {uri}...")
    
    try:
        async with websockets.connect(uri) as websocket:
            print("Connected to ROS WebSocket server!")
            
            # Setup signal handling for graceful exit
            loop = asyncio.get_event_loop()
            for sig in (signal.SIGINT, signal.SIGTERM):
                loop.add_signal_handler(sig, lambda: asyncio.create_task(shutdown(websocket, loop)))
            
            # Send a test message
            test_message = {
                "msg_id": "test-001",
                "command": "ping",
                "data": "Hello from test client"
            }
            
            print(f"Sending test message: {test_message}")
            await websocket.send(json.dumps(test_message))
            
            # Listen for messages from the server
            while True:
                try:
                    message = await websocket.recv()
                    data = json.loads(message)
                    print(f"Received: {data}")
                    
                    # Check if it's an acknowledgment
                    if "status" in data and data.get("status") == "ok":
                        print("✓ Server acknowledged our message")
                    
                    # If it's mesh data or other types
                    if "type" in data and data.get("type") == "mesh_data":
                        print("✓ Received mesh data from ROS")
                        print(f"  Timestamp: {data.get('timestamp')}")
                        
                except json.JSONDecodeError:
                    print(f"Received non-JSON message: {message}")
                
    except websockets.exceptions.ConnectionClosedError as e:
        print(f"Connection closed with error: {e}")
    except websockets.exceptions.InvalidStatusCode as e:
        print(f"Failed to connect: {e}")
    except ConnectionRefusedError:
        print(f"Connection refused. Make sure the server is running at {SERVER_HOST}:{SERVER_PORT}")
    except Exception as e:
        print(f"An error occurred: {e}")

async def shutdown(websocket, loop):
    print("\nShutting down...")
    await websocket.close()
    tasks = [t for t in asyncio.all_tasks() if t is not asyncio.current_task()]
    for task in tasks:
        task.cancel()
    await asyncio.gather(*tasks, return_exceptions=True)
    loop.stop()

if __name__ == "__main__":
    # Allow command line arguments to override defaults
    if len(sys.argv) > 1:
        SERVER_HOST = sys.argv[1]
    if len(sys.argv) > 2:
        SERVER_PORT = int(sys.argv[2])
    
    print("WebSocket Test Client for ROS Server")
    print(f"Target: {SERVER_HOST}:{SERVER_PORT}")
    print("Press Ctrl+C to exit")
    
    try:
        asyncio.run(test_connection())
    except KeyboardInterrupt:
        print("\nExiting...")