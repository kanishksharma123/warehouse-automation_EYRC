#!/bin/bash

# Store URLs in two variables
URL1="http://www.hivemq.com/demos/websocket-client/"
URL2="https://docs.google.com/spreadsheets/d/1gmrC1sCzUAQ_NO2QHd9YSQvl0fSJT-nAUC2LL2ZwPj8/edit#gid=684443896"

# Print some message
echo "** Opening $URL2 in Firefox **"

# Use firefox to open the two URLs in separate windows
firefox -new-window $URL2

