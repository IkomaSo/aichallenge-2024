# マルチキャスト有効化とバッファサイズ変更用bashスクリプト
# configure the network buffers to the maximum value
# executed when print "ros2: failed to increase socket receive buffer size to at least 10485760 bytes, current is 425984 bytes"
sudo ip link set multicast on lo
sudo sysctl -w net.core.rmem_max=2147483647 >/dev/null
