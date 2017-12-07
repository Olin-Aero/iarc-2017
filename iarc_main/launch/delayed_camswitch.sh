#!/bin/bash

sleep 5

rosservice call --wait /ardrone/setcamchannel "channel: 1"
