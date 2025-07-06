#!/bin/bash

for x in {10..100}; do
    # Export the display
    export DISPLAY=localhost:${x}.0
    
    # Try to run rqt in background
    rqt &
    RQT_PID=$!
    
    # Give it some time to start (adjust as needed)
    sleep 3
    
    # Check if rqt is still running
    if ps -p $RQT_PID > /dev/null; then
        # If running, it means it succeeded
        kill $RQT_PID
        wait $RQT_PID 2>/dev/null
        echo "Successful display: $x"
        exit 0
    fi
done

echo "No working display found in range 10-100"
exit 1