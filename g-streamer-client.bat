set path=%~dp0

for /f "delims=" %%R in ("%path%") do set FILE_URL=%%~fR%
set FILE_URL=%FILE_URL:///\\=//%
set FILE_URL=%FILE_URL:\=/%

cd C:\gstreamer\1.0\x86_64\bin 
gst-launch-1.0.exe -vvv -e udpsrc port=5805 ! "application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96" ! rtph264depay ! avdec_h264 ! gdkpixbufoverlay location=%FILE_URL%overlay.png ! fpsdisplaysink async=false
pause