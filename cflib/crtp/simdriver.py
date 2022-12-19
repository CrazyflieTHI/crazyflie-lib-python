# -*- coding: utf-8 -*-
# -----------------------------------------------------------------------------
# This file is part of the Augsburg Crazyflie Project.
# (C) 2022 Hochschule Augsburg, University of Applied Sciences
# -----------------------------------------------------------------------------
#
# Company:        University of Applied Sciences, Augsburg, Germany
# Author:         Thomas Izycki <thomas.izycki2@hs-augsburg.de>
#
# Description:    Crazyflie CRTP link driver intended for the use with the
#                 ROS Gazebo sim_cf simulation.
#
# --------------------- LICENSE -----------------------------------------------
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program; if not, see <http://www.gnu.org/licenses/>
# or write to the Free Software Foundation, Inc.,
# 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
# -----------------------------------------------------------------------------
"""
Crazyflie simulation CRTP link driver.

This driver is used to communicate with an emulated Crazyflie in ROS Gazebo
sim_cf software-in-the-loop simulation.
"""
import logging
import queue
import struct
import posix_ipc
from threading import Thread
from multiprocessing import Queue

from cflib.utils.callbacks import Caller
from cflib.crtp.crtpstack import CRTPPacket
from cflib.crtp.crtpdriver import CRTPDriver

__author__ = 'Thomas Izycki'
__all__ = ['SimDriver']

logger = logging.getLogger(__name__)

PORT_IPC_CRTP = 0x0

linkUris = {
    0xE7E7E7E701:'radio://0/80/2M/E7E7E7E701',
    0xE7E7E7E702:'radio://0/80/2M/E7E7E7E702',
    0xE7E7E7E703:'radio://0/80/2M/E7E7E7E703',
    0xE7E7E7E704:'radio://0/80/2M/E7E7E7E704',
    0xE7E7E7E705:'radio://0/80/2M/E7E7E7E705',
    0xE7E7E7E706:'radio://0/80/2M/E7E7E7E706',
    0xE7E7E7E707:'radio://0/80/2M/E7E7E7E707',
    0xE7E7E7E708:'radio://0/80/2M/E7E7E7E708',
    0xE7E7E7E709:'radio://0/80/2M/E7E7E7E709',
    0xE7E7E7E70A:'radio://0/80/2M/E7E7E7E70A'
}

class SimDriver(CRTPDriver):
    """ Simulation link driver """

    def __init__(self):
        """ Create the link driver """
        CRTPDriver.__init__(self)
        self.uri = ''
        self.crtpCom = None
        self.link_error_callback = None
        self.link_quality_callback = None
        self._in_queue = None
        self.needs_resending = False

        # Random version
        self.version = 4.0

    def connect(self, uri, link_quality_callback, link_error_callback):
        """Connect the driver to a specified URI

            @param uri Uri of the link to open
            @param link_quality_callback Callback to report link quality in percent
            @param link_error_callback Callback to report errors (will result in
                disconnection)
        """
        self.link_error_callback = link_error_callback
        self.link_quality_callback = link_quality_callback

        # Inter-thread communication
        self._in_queue = queue.Queue()

        # Inter-process communication
        rxIpcQueueName = "/rxsimcrtpmq" + uri[-2:]
        txIpcQueueName = "/txsimcrtpmq" + uri[-2:]
        # print(f"rxIpcQueueName {rxIpcQueueName} txIpcQueueName {txIpcQueueName}")
        self.crtpCom = SimulationIpcPosix(rxIpcQueueName, txIpcQueueName)
        self.crtpCom.incomingCrtpPacket.add_callback(self._receive_packet_posix_mq)

    def _receive_packet_posix_mq(self, crtp_packet):
        self._in_queue.put(crtp_packet)

    def send_packet(self, pk):
        """Send a CRTP packet"""
        self.crtpCom.sendCrtpPacket(pk)

    def receive_packet(self, wait=0):
        """
        Receive a packet though the link. This call is blocking but will
        timeout and return None if a timeout is supplied.
        """
        if wait == 0:
            try:
                return self._in_queue.get(False)
            except queue.Empty:
                return None
        elif wait < 0:
            try:
                return self._in_queue.get(True)
            except queue.Empty:
                return None
        else:
            try:
                return self._in_queue.get(True, wait)
            except queue.Empty:
                return None

    def get_status(self):
        return 'Simulation link driver version {}'.format(self.version)

    def get_name(self):
        return 'Simulation link driver'

    def scan_interface(self, address=None):
        """
        Scan interface for available Crazyflie quadcopters and return a list
        with them.
        """
        if address is None:
            return []
        else:
            return [[linkUris[address], '']]

    def close(self):
        """Close the link"""
        self.crtpCom.endCommunication()


class InterProcessPacket:
    def __init__(self, port, payload):
        self.port = port
        self.payload = payload

    def getPort(self):
        return self.port

    def getPayload(self):
        return self.payload

    def getBytes(self):
        '''Requires payload to be already of type bytearray'''
        if(type(self.payload) != bytearray):
            print("ERROR InterProcessPacket: Payload needs to be of type bytearray.")
            return None
        packetBytes = bytearray()

        portBytes = struct.pack('<B', self.port)
        packetBytes.extend(portBytes)
        packetBytes.extend(self.payload)
        return packetBytes


class SimulationIpcPosix():
    def __init__(self, rxQueueName, txQueueName):
        self.incomingCrtpPacket = Caller()
        self.incomingControlPacket = Caller()

        self.rxQueueName = rxQueueName
        self.txQueueName = txQueueName

        # Inter-process communication (IPC) with POSIX message queues
        try:
            self.rxIpcQueue = posix_ipc.MessageQueue(rxQueueName, flags=posix_ipc.O_CREAT,
                                                     max_messages=100, max_message_size=33)
        except Exception as e:
            print(f"ERROR SimulationIpcPosix: {rxQueueName} creation failed. "+ str(e))
            return

        try:
            self.txIpcQueue = posix_ipc.MessageQueue(txQueueName, flags=posix_ipc.O_CREAT,
                                                     max_messages=100, max_message_size=33)
        except Exception as e:
            print(f"ERROR SimulationIpcPosix: {txQueueName} creation failed. "+ str(e))
            return

        self.simCom = InterProcessCommunicatorPosix(self.txIpcQueue, self.rxIpcQueue)
        self.simCom.addPortCallback(PORT_IPC_CRTP, self._handleIncomingCrtpPacket)

    def getIpcQueueNames(self):
        '''Return the names of the rx and tx posix queues in this order'''
        return self.rxQueueName, self.txQueueName

    def sendCrtpPacket(self, crtpPacket):
        crtpPacketBytes = self.convertCRTPPacketObjectToBytearray(crtpPacket)
        self.sendCrtpPacketBytes(crtpPacketBytes)

    def sendCrtpPacketBytes(self, crtpPacketBytes):
        ''' Send a CRTP packet via POSIX message queue

        params:
        crtpPacketBytes -> bytearray: The CRTP packet of type bytearray
        '''
        self.simCom.send(PORT_IPC_CRTP, crtpPacketBytes)

    def sendCrtpPacketRaw(self, channel, payload, port=0x09):
        '''Create a CRTP packet from channel, payload and port and 
           convert it to a bytearray for sending it via POSIX message
           queue to another process.

        params:
        channel -> int: CRTP channel
        payload -> bytearray: The payload for the CRTP packet
        port -> int: CRTP port
        '''
        crtpPacket = self.createCRTPPacket(port, channel, payload)
        crtpPacketBytes = self.convertCRTPPacketObjectToBytearray(crtpPacket)
        self.sendCrtpPacket(crtpPacketBytes)

    def createCRTPPacket(self, port, channel, payload):
        if isinstance(payload, int):
            pk = CRTPPacket()
            pk.data = struct.pack('<I', payload)
        else:
            pk = CRTPPacket(data=payload)

        pk.set_header(port, channel)
        return pk

    def convertCRTPPacketObjectToBytearray(self, crtp_packet):
        MAX_PAYLOAD = 30
        packet = bytearray()
        padding = bytearray()

        # Packet size is the size of data payload
        packet_size    = crtp_packet.get_data_size()
        packet_data    = crtp_packet._get_data()
        packet_header  = crtp_packet.get_header()

        header = struct.pack('<B', packet_header )
        size   = struct.pack('<B', packet_size)
        for i in range(MAX_PAYLOAD - packet_size):
            padding.extend(bytes(0x1))

        packet.extend(size)
        packet.extend(header)
        packet.extend(packet_data)
        packet.extend(padding)
        return packet

    def _handleIncomingCrtpPacket(self, crtpPacketBytes):
        crtpPacket = self.convertBytearrayToCRTPPacketObject(crtpPacketBytes)
        self.incomingCrtpPacket.call(crtpPacket)

    def convertBytearrayToCRTPPacketObject(self, crtpPacketBytes):
        # Extract the header, size and data from the bytearray
        # and save in a newly created CRTP packet
        packet_size   = struct.unpack('<B', crtpPacketBytes[:1])
        packet_header = struct.unpack('<B', crtpPacketBytes[1:2])
        crtp_packet   = CRTPPacket(header=packet_header[0], data=crtpPacketBytes[2:(packet_size[0]+2)])

        return crtp_packet

    def endCommunication(self):
        self.simCom.endCommunication()
        self.rxIpcQueue.unlink()
        self.txIpcQueue.unlink()


class InterProcessCommunicatorPosix:
    '''
    Inter process communication. Uses the InterProcessPacket to
    send and receive data on user-defined ports.

    :param txQueue: multiprocessing.Queue for transmitting data
    :param rxQueue: multiprocessing.Queue for receiving data

    Remember, the txQueue of the first process is the rxQueue
    of the second process and vice versa.
    '''
    def __init__(self, txQueue, rxQueue):
        self.txQueue = txQueue
        self.rxQueue = rxQueue

        self.txThreadQueue = Queue()

        # The port is the key associated to a
        # list of registered callback functions
        self.portCallbacks = {}

        self.rxData = True
        self.rxThread = Thread(target = self._receiveIpcPacket)
        self.txData = True
        self.txThread = Thread(target=self._sendIpcPacket)
        self.rxThread.start()
        self.txThread.start()

    def _receiveIpcPacket(self):
        while(self.rxData):
            try:
                # Wait for a packet to arrive
                ipcPacketBytes = self.rxQueue.receive(timeout=0.1)

                # ipcPacketPrio = ipcPacketBytes[1]
                # print(f"Packet prio: {ipcPacketPrio}")
                ipcPacketPort = struct.unpack('<B', ipcPacketBytes[0][:1])[0]
                # print(f"ipcPacketBytes: {ipcPacketBytes}")
                # print(f"crtpBytes: {ipcPacketBytes[0][1:]}")

                self._invokePortCallbacks(ipcPacketPort, ipcPacketBytes[0][1:])
            except:
                pass

    def addPortCallback(self, port, cb):
        ''' Add a callback function for receiving data on the specified port

        :param port: Port on which the InterProcessPacket arrives
        :param cb: Callback function to be called when data arrives.
                   The function needs to take one parameter for
                   accepting the payload.
        '''
        # Check if the callback is already registered on the port
        if port in self.portCallbacks:
            for currentCb in self.portCallbacks.items():
                # Do not register duplicates
                if currentCb is cb:
                    return
        # If there are no callbacks on the port, initialize the port
        if port not in self.portCallbacks:
            self.portCallbacks[port] = []

        # Finally register the callback
        self.portCallbacks[port].append(cb)

    def _invokePortCallbacks(self, port, payload):
        """ Call the registered callbacks """
        if port not in self.portCallbacks:
            return
        copyOfCallbacks = list(self.portCallbacks[port])
        for cb in copyOfCallbacks:
            cb(payload)

    def removePortCallback(self, port, cb):
        # Check if the callback is registered on the port
        if port in cb:
            for currentCb in self.portCallbacks.items():
                # Remove the callback from the list
                if currentCb is cb:
                    self.portCallbacks.remove(cb)

    def send(self, port, payload):
        ipcPacket = InterProcessPacket(port, payload)
        self.txThreadQueue.put(ipcPacket)

    def _sendIpcPacket(self):
        while(self.txData):
            try:
                ipcPacket = self.txThreadQueue.get(block=True, timeout=0.1)
                self.txQueue.send(ipcPacket.getBytes())
            except:
                pass

    def endCommunication(self):
        self.rxData = False
        self.txData = False
        return self.rxThread
