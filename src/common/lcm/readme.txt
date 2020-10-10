if cannot send message to other machine, should
export LCM_DEFAULT_URL=udpm://239.255.76.67:7667?ttl=1

if cannot send message within the host machine, should
ifconfig lo multicast
route add -net 224.0.0.0 netmask 240.0.0.0 dev lo