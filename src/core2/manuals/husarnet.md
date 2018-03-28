---
title: 'Husarnet'
platform: 'CORE2'
autotoc: true
layout: layout.hbs
order: 6
page: 'Manuals'
onepager: true
---

Husarnet is a P2P, VPN network dedicated for robotics. Thanks to Husarnet you can connect your robots, server and laptop to a single network that is independent from any external infrastructure. All traffic goes directly between your robots. 

A single Husarnet network is called "Virtual Robot". Virtual Robot can be combined of one, or a few robots, your laptop running user interface and a server processing navigation algorithm for all robots at once. You are no longer limited by processing power of your robot. You no longer need to care about how to create your robot swarm. Husarnet does it for you.

# Architecture #
## Common solutions ##

Standard solution to connect devices over Internet uses client-server architecture. This architecture seems to be simple, but robot creator has to be aware of some drawbacks: from a user point of view:

- users can be spied by manufacturer
- devices will cease to work when manufacturer stops supporting the servers
- devices need internet access to function, LAN doesn't suffice
- medium to large latency  in communication between elements connected to the system

<div><center><img src="https://raw.githubusercontent.com/husarion/static_docs/master/src/assets/img/husarnet/manual/cloud_bad.jpg" height="50%" width="50%"></center></div>

## Solution provided by Husarnet ##

Husarnet instead of client-server, uses P2P paradigm. Central server (Husarion cloud) only helps establish connections over the Internet, LAN connections may be established even without it.

Husarnet features:

- node identity is hash of its (Curve25519) public key
- node identity is also its global IPv6 address (fc94::/16 network)
- connection is authenticated by this identifier
- you can call hosts in a single Husarnet network using their hostname
- Internet traffic goes directly between Virtual Robot elements, so latency is lower than in case of client-server architecture

<div><center><img src="https://raw.githubusercontent.com/husarion/static_docs/master/src/assets/img/husarnet/manual/husarnet_diagram.jpeg" height="50%" width="50%"></center></div>

# Quick start guide#


## Install Husarnet ##

Execute the following steps:
1. Install Husarnet: `$ curl https://files.husarion.com/install/husarnet.sh | sudo bash`
2. Restart Husarnet: `$ systemctl restart husarnet`
3. Link your device to Husarnet: `$ sudo husarnet websetup`
4. Click the link which will be shown by this command.
<div><center><img src="https://raw.githubusercontent.com/husarion/static_docs/master/src/assets/img/husarnet/manual/husarnet_websetup.PNG" height="50%" width="50%"></center></div>
5. Fill-in the "Add device to Husarnet" form. If this is you first device, you should select "Create new virtual robot" in "Join virtual robot". Otherwise, you may use one of your previous virtual robots.

<div><center><img src="https://raw.githubusercontent.com/husarion/static_docs/master/src/assets/img/husarnet/manual/connect_linux.png" height="50%" width="50%"></center></div>

## Install ROS on Ubuntu 16.04 ##

(This section is based on http://wiki.ros.org/kinetic/Installation/Ubuntu )
1. Execute the following commands to add ROS repository:
```
$ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros.list
$ sudo apt-get update
```
2. Install the packages you need, for example:
```
$ sudo apt-get install -y ros-kinetic-find-object-2d
```
3. **Important:** Add this line to .bashrc (or .zshrc if you use zsh) of the user who will use ROS:
```
export ROS_IPV6=on
export ROS_MASTER_URI=http://master:11311
```

# Using Husarnet #

1. You may contact all other devices in your virtual robot just by using their hostnames, e.g.:
```
$ ping6 mydevice1
$ ssh mydevice1
$ wget http://mydevice:8000
```
2. You may check network status using: 

```
$ sudo husarnet status
Husarnet IP address: fc94:xxxx:xxxx:xxxx:xxxx:xxxx:xxxx:xxxx
UDP connection to base: [::ffff:188.165.23.196]:5582

Peer fc94:b57c:c306:595f:9933:320a:a77:bffa
  target=[::ffff:192.168.1.45]:5582
  secure connection established
Peer fc94:a1e4:7b6b:3222:b1f0:90fa:e41f:9857
  tunneled
  secure connection established
```

In this example, you are connected to the first peer directly (fc94:...:bffa) via local network (192.168.1.45). Direct connection to second peer could not be established (tunneled) - this probably means that network you are using blocks UDP traffic. Ensure the firewall allows outgoing UDP traffic, at least on port 5582.
3. Just be aware that the servers and client you are using must support IPv6 (as Husarnet is an IPv6 overlay network) - for example, you have to listen on "::", not "0.0.0.0".


# Managing Husarnet manually #


Sometimes managing the devices via Husarion Cloud can be cumbersome. You can skip connecting your device to the cloud and manage whitelist and hostnames via command line.

If not the whitelist, you could reach any device connected to Husarnet without any configuration. If that suits you, simply disable it on all devices your have `husarnet whitelist disable`. Be aware of security implications of this action (e.g. do this only if you are confident that your firewall is strong enough).

Otherwise, whitelist has to contain IP addresses of the devices that are authorized to connect to your host. You can manage it using two commands:

- `husarnet whitelist add [address]` - Add fc94 IP address to the whitelist.
- `husarnet whitelist rm [address]` - Remove fc94 IP address from the whitelist.

If you want A to communicate with B, make sure to add A to B whitelist and B to A whitelist.

# How connections are established? ##

- First, the Husarnet client connects to the base server (via TCP on port 443 and optionally UDP on port 5582) hosted by Husarion.
- Initially the encrypted data is tunnelled via the base server.
- The devices attempt to connect to local IP addresses (retrieved via the base server). This will succeed if they are in the same network or one of them has public IP address (and UDP is not blocked).
- The devices attempt to perform NAT traversal assisted by the base server. This will succeed if NAT is not symmetric and UDP is not blocked on the firewall.
- The devices send multicast discovery to the local network. This will succeed if the devices are on the same network (even if there is no internet connectivity or the base server can’t be reached).

# Husarnet security #

Security was of an uttermost importance when designing Husarnet. 

Cryptography: Husarnet uses X25519 from libsodium for key exchange, with ephemeral Curve25519 keys for forward secrecy. The packets are encrypted using libsodium ChaCha20-Poly1305 secretbox construction with random 192-bit nonce.

Runtime safety: Husarnet is written in C++ using modern memory-safe constructs. Linux version drops all capabilities after initializing. It retains access to /etc/hosts via a helper process.

If Husarnet instance is not connected to the Husarion Cloud, the whitelist and /etc/hosts can only be changed by local root user. Otherwise, the owner of the Husarion Cloud account can also influence the configuration by adding the device to virtual robots/networks.

