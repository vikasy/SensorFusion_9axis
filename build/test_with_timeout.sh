#!/bin/bash
./bin/sensor_fusion &
PID=$!
sleep 5
kill $PID 2>/dev/null
wait $PID 2>/dev/null
echo "Test completed with timeout"
