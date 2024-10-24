/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Siddharth Bharat Purohit
 */

#include <AP_HAL/AP_HAL.h>

#if HAL_ENABLE_DRONECAN_DRIVERS

#include "AP_DroneCAN_DNA_Server.h"
#include "AP_DroneCAN.h"
#include <StorageManager/StorageManager.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <stdio.h>
extern const AP_HAL::HAL& hal;

// FORMAT REVISION DREAMS (things to address if the nodedata needs to be changed substantially)
// * have DNA server accept only a 16 byte local UID to avoid overhead from variable sized hash
// * have a real empty flag for entries and/or use a CRC which is not zero for an input of all zeros

#define NODEDATA_MAGIC 0xAC01
#define NODEDATA_MAGIC_LEN 2 // uint16_t
#define MAX_NODE_ID    125
#define NODEDATA_LOC(node_id) ((node_id * sizeof(struct NodeData)) + NODEDATA_MAGIC_LEN)

#define debug_dronecan(level_debug, fmt, args...) do { AP::can().log_text(level_debug, "DroneCAN", fmt, ##args); } while (0)

// database is currently shared by all DNA servers
AP_DroneCAN_DNA_Server::Database AP_DroneCAN_DNA_Server::db;

AP_DroneCAN_DNA_Server::AP_DroneCAN_DNA_Server(AP_DroneCAN &ap_dronecan, CanardInterface &canard_iface, uint8_t driver_index) :
    _ap_dronecan(ap_dronecan),
    _canard_iface(canard_iface),
    storage(StorageManager::StorageCANDNA),
    allocation_sub(allocation_cb, driver_index),
    node_status_sub(node_status_cb, driver_index),
    node_info_client(_canard_iface, node_info_cb) {}

/* Method to generate 6byte hash from the Unique ID.
We return it packed inside the referenced NodeData structure */
void AP_DroneCAN_DNA_Server::Database::getHash(NodeData &node_data, const uint8_t unique_id[], uint8_t size) const
{
    uint64_t hash = FNV_1_OFFSET_BASIS_64;
    hash_fnv_1a(size, unique_id, &hash);

    // xor-folding per http://www.isthe.com/chongo/tech/comp/fnv/
    hash = (hash>>56) ^ (hash&(((uint64_t)1<<56)-1));

    // write it to ret
    for (uint8_t i=0; i<6; i++) {
        node_data.hwid_hash[i] = (hash >> (8*i)) & 0xff;
    }
}

//Read Node Data from Storage Region
void AP_DroneCAN_DNA_Server::Database::readNodeData(NodeData &data, uint8_t node_id)
{
    if (node_id > MAX_NODE_ID) {
        return;
    }

    storage->read_block(&data, NODEDATA_LOC(node_id), sizeof(struct NodeData));
}

//Write Node Data to Storage Region
void AP_DroneCAN_DNA_Server::Database::writeNodeData(const NodeData &data, uint8_t node_id)
{
    if (node_id > MAX_NODE_ID) {
        return;
    }

    storage->write_block(NODEDATA_LOC(node_id), &data, sizeof(struct NodeData));
}

/* Remove Node Data from Server Record in Storage,
and also clear Occupation Mask */
void AP_DroneCAN_DNA_Server::Database::freeNodeID(uint8_t node_id)
{
    if (node_id > MAX_NODE_ID) {
        return;
    }

    struct NodeData node_data;

    //Eliminate from Server Record
    memset(&node_data, 0, sizeof(node_data));
    writeNodeData(node_data, node_id);

    //Clear Occupation Mask
    node_storage_occupied.clear(node_id);
}

/* Go through Server Records, and fetch node id that matches the provided
Unique IDs hash.
Returns 0 if no Node ID was detected */
uint8_t AP_DroneCAN_DNA_Server::Database::getNodeIDForUniqueID(const uint8_t unique_id[], uint8_t size)
{
    NodeData node_data, cmp_node_data;
    getHash(cmp_node_data, unique_id, size);

    for (int i = MAX_NODE_ID; i > 0; i--) {
        if (node_storage_occupied.get(i)) {
            readNodeData(node_data, i);
            if (memcmp(node_data.hwid_hash, cmp_node_data.hwid_hash, sizeof(NodeData::hwid_hash)) == 0) {
                return i; // node ID found
            }
        }
    }
    return 0; // not found
}

/* Hash the Unique ID and add it to the Server Record
for specified Node ID. */
void AP_DroneCAN_DNA_Server::Database::addNodeIDForUniqueID(uint8_t node_id, const uint8_t unique_id[], uint8_t size)
{
    NodeData node_data;
    getHash(node_data, unique_id, size);
    //Generate CRC for validating the data when read back
    node_data.crc = crc_crc8(node_data.hwid_hash, sizeof(node_data.hwid_hash));

    //Write Data to the records
    writeNodeData(node_data, node_id);

    node_storage_occupied.set(node_id);
}

//Checks if a valid Server Record is present for specified Node ID
bool AP_DroneCAN_DNA_Server::Database::isValidNodeDataAvailable(uint8_t node_id)
{
    NodeData node_data;
    readNodeData(node_data, node_id);

    uint8_t empty_hwid[sizeof(NodeData::hwid_hash)] = {0};
    uint8_t crc = crc_crc8(node_data.hwid_hash, sizeof(node_data.hwid_hash));
    if (crc == node_data.crc && memcmp(&node_data.hwid_hash[0], &empty_hwid[0], sizeof(empty_hwid)) != 0) {
        return true;
    }
    return false;
}

// initialize database (storage accessor is always replaced with the one supplied)
void AP_DroneCAN_DNA_Server::Database::init(StorageAccess *storage_)
{
    // storage size must be synced with StorageCANDNA entry in StorageManager.cpp
    static_assert(NODEDATA_LOC(MAX_NODE_ID+1) <= 1024, "DNA storage too small");

    // might be called from multiple threads if multiple servers use the same database
    WITH_SEMAPHORE(sem);

    storage = storage_; // use supplied accessor

    // validate magic number
    uint16_t magic = storage->read_uint16(0);
    if (magic != NODEDATA_MAGIC) {
        reset(); // re-initializing the database will put the magic back
    }

    /* Go through our records and look for valid NodeData, to initialise
    occupied status */
    for (uint8_t i = 1; i <= MAX_NODE_ID; i++) {
        if (isValidNodeDataAvailable(i)) {
            node_storage_occupied.set(i);
        }
    }
}

/* Initialises Publishers for respective UAVCAN Instance
Also resets the Server Record in case there is a mismatch
between specified node id and unique id against the existing
Server Record. */
bool AP_DroneCAN_DNA_Server::init(uint8_t own_unique_id[], uint8_t own_unique_id_len, uint8_t node_id)
{
    //Read the details from AP_DroneCAN
    server_state = HEALTHY;

    db.init(&storage); // initialize the database with our accessor

    if (_ap_dronecan.check_and_reset_option(AP_DroneCAN::Options::DNA_CLEAR_DATABASE)) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "UC DNA database reset");
        db.reset();
    }

    db.initServer(node_id, own_unique_id, own_unique_id_len);

    /* Also add to seen node id this is to verify
    if any duplicates are on the bus carrying our Node ID */
    node_seen.set(node_id);
    node_verified.set(node_id);
    node_healthy.set(node_id);
    self_node_id = node_id;
    return true;
}

// handle initializing the server with the given expected node ID and unique ID
void AP_DroneCAN_DNA_Server::Database::initServer(uint8_t node_id, const uint8_t own_unique_id[], uint8_t own_unique_id_len)
{
    WITH_SEMAPHORE(sem);

    // Making sure that the server is started with the same node ID
    const uint8_t stored_own_node_id = getNodeIDForUniqueID(own_unique_id, own_unique_id_len);
    static bool reset_done;
    if (stored_own_node_id != node_id) { // cannot match if not found
        // We have no matching record of our own Unique ID do a reset
        if (!reset_done) {
            /* ensure we only reset once per power cycle
            else we will wipe own record on next init(s) */
            reset();
            reset_done = true;
        }
        //Add ourselves to the Server Record
        addNodeIDForUniqueID(node_id, own_unique_id, own_unique_id_len);
    }
}


//Reset the Server Records
void AP_DroneCAN_DNA_Server::Database::reset()
{
    WITH_SEMAPHORE(sem);

    NodeData node_data;
    memset(&node_data, 0, sizeof(node_data));
    node_storage_occupied.clearall();

    //Just write empty Node Data to the Records
    // ensure node ID 0 is cleared even if we can't use it so we know the state
    for (uint8_t i = 0; i <= MAX_NODE_ID; i++) {
        writeNodeData(node_data, i);
    }

    //Ensure we mark magic at the end
    storage->write_uint16(0, NODEDATA_MAGIC);
}

/* Go through the Occupation mask for available Node ID
based on pseudo code provided in
uavcan/protocol/dynamic_node_id/1.Allocation.uavcan */
uint8_t AP_DroneCAN_DNA_Server::Database::findFreeNodeID(uint8_t preferred)
{
    if (preferred == 0) {
        preferred = 125;
    }
    // Search up
    uint8_t candidate = preferred;
    while (candidate <= 125) {
        if (!node_storage_occupied.get(candidate)) {
            return candidate;
        }
        candidate++;
    }
    //Search down
    candidate = preferred;
    while (candidate > 0) {
        if (!node_storage_occupied.get(candidate)) {
            return candidate;
        }
        candidate--;
    }
    // Not found
    return 0;
}

/* Run through the list of seen node ids for verification no more
than once per 5 second. We continually verify the nodes in our 
seen list, So that we can raise issue if there are duplicates
on the bus. */
void AP_DroneCAN_DNA_Server::verify_nodes()
{
    uint32_t now = AP_HAL::millis();
    if ((now - last_verification_request) < 5000) {
        return;
    }

#if HAL_LOGGING_ENABLED
    uint8_t log_count = AP::logger().get_log_start_count();
    if (log_count != last_logging_count) {
        last_logging_count = log_count;
        node_logged.clearall();
    }
#endif

    //Check if we got acknowledgement from previous request
    //except for requests using our own node_id
    if (curr_verifying_node == self_node_id) {
        nodeInfo_resp_rcvd = true;
    }

    if (!nodeInfo_resp_rcvd) {
        /* Also notify GCS about this
        Reason for this could be either the node was disconnected
        Or a node with conflicting ID appeared and is sending response
        at the same time. */
        node_verified.clear(curr_verifying_node);
    }

    last_verification_request = now;
    //Find the next registered Node ID to be verified.
    for (uint8_t i = 0; i <= MAX_NODE_ID; i++) {
        curr_verifying_node = (curr_verifying_node + 1) % (MAX_NODE_ID + 1);
        if ((curr_verifying_node == self_node_id) || (curr_verifying_node == 0)) {
            continue;
        }
        if (node_seen.get(curr_verifying_node)) {
            break;
        }
    }
    if (db.isOccupied(curr_verifying_node)) {
        uavcan_protocol_GetNodeInfoRequest request;
        node_info_client.request(curr_verifying_node, request);
        nodeInfo_resp_rcvd = false;
    }
}

/* Handles Node Status Message, adds to the Seen Node list
Also starts the Service call for Node Info to complete the
Verification process. */
void AP_DroneCAN_DNA_Server::handleNodeStatus(const CanardRxTransfer& transfer, const uavcan_protocol_NodeStatus& msg)
{
    if (transfer.source_node_id > MAX_NODE_ID || transfer.source_node_id == 0) {
        return;
    }
    if ((msg.health != UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK ||
        msg.mode != UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL) &&
        !_ap_dronecan.option_is_set(AP_DroneCAN::Options::DNA_IGNORE_UNHEALTHY_NODE)) {
        //if node is not healthy or operational, clear resp health mask, and set fault_node_id
        fault_node_id = transfer.source_node_id;
        server_state = NODE_STATUS_UNHEALTHY;
        node_healthy.clear(transfer.source_node_id);
    } else {
        node_healthy.set(transfer.source_node_id);
        if (node_healthy == node_verified) {
            server_state = HEALTHY;
        }
    }
    if (!node_verified.get(transfer.source_node_id)) {
        //immediately begin verification of the node_id
        uavcan_protocol_GetNodeInfoRequest request;
        node_info_client.request(transfer.source_node_id, request);
    }
    //Add node to seen list if not seen before
    node_seen.set(transfer.source_node_id);
}

/* Node Info message handler
Handle responses from GetNodeInfo Request. We verify the node info
against our records. Marks Verification mask if already recorded,
Or register if the node id is available and not recorded for the
received Unique ID */
void AP_DroneCAN_DNA_Server::handleNodeInfo(const CanardRxTransfer& transfer, const uavcan_protocol_GetNodeInfoResponse& rsp)
{
    if (transfer.source_node_id > MAX_NODE_ID || transfer.source_node_id == 0) {
        return;
    }
    /*
      if we haven't logged this node then log it now
     */
#if HAL_LOGGING_ENABLED
    if (!node_logged.get(transfer.source_node_id) && AP::logger().logging_started()) {
        node_logged.set(transfer.source_node_id);
        uint64_t uid[2];
        memcpy(uid, rsp.hardware_version.unique_id, sizeof(rsp.hardware_version.unique_id));
        // @LoggerMessage: CAND
        // @Description: Info from GetNodeInfo request
        // @Field: TimeUS: Time since system startup
        // @Field: Driver: Driver index
        // @Field: NodeId: Node ID
        // @Field: UID1: Hardware ID, part 1
        // @Field: UID2: Hardware ID, part 2
        // @Field: Name: Name string
        // @Field: Major: major revision id
        // @Field: Minor: minor revision id
        // @Field: Version: AP_Periph git hash
        AP::logger().Write("CAND", "TimeUS,Driver,NodeId,UID1,UID2,Name,Major,Minor,Version",
                           "s-#------", "F--------", "QBBQQZBBI",
                           AP_HAL::micros64(),
                           _ap_dronecan.get_driver_index(),
                           transfer.source_node_id,
                           uid[0], uid[1],
                           rsp.name.data,
                           rsp.software_version.major,
                           rsp.software_version.minor,
                           rsp.software_version.vcs_commit);
    }
#endif

    bool duplicate = db.handleNodeInfo(transfer.source_node_id, rsp.hardware_version.unique_id);
    if (duplicate) {
        if (!_ap_dronecan.option_is_set(AP_DroneCAN::Options::DNA_IGNORE_DUPLICATE_NODE)) {
            /* This is a device with node_id already registered
            for another device */
            server_state = DUPLICATE_NODES;
            fault_node_id = transfer.source_node_id;
            memcpy(fault_node_name, rsp.name.data, sizeof(fault_node_name));
        }
    } else {
        //Verify as well
        node_verified.set(transfer.source_node_id);
        if (transfer.source_node_id == curr_verifying_node) {
            nodeInfo_resp_rcvd = true;
        }
    }
}

// handle processing the node info message. returns true if duplicate.
bool AP_DroneCAN_DNA_Server::Database::handleNodeInfo(uint8_t source_node_id, const uint8_t unique_id[])
{
    WITH_SEMAPHORE(sem);

    if (isOccupied(source_node_id)) {
        //if node_id already registered, just verify if Unique ID matches as well
        if (source_node_id == getNodeIDForUniqueID(unique_id, 16)) {
            return false;
        } else {
            /* This is a device with node_id already registered
            for another device */
            return true;
        }
    } else {
        /* Node Id was not allocated by us, or during this boot, let's register this in our records
        Check if we allocated this Node before */
        uint8_t prev_node_id = getNodeIDForUniqueID(unique_id, 16);
        if (prev_node_id != 0) {
            //yes we did, remove this registration
            freeNodeID(prev_node_id);
        }
        //add a new server record
        addNodeIDForUniqueID(source_node_id, unique_id, 16);
        return false;
    }
}

/* Handle the allocation message from the devices supporting
dynamic node allocation. */
void AP_DroneCAN_DNA_Server::handleAllocation(const CanardRxTransfer& transfer, const uavcan_protocol_dynamic_node_id_Allocation& msg)
{
    if (transfer.source_node_id != 0) {
        //Ignore Allocation messages that are not DNA requests
        return;
    }
    uint32_t now = AP_HAL::millis();

    if (rcvd_unique_id_offset == 0 ||
        (now - last_alloc_msg_ms) > 500) {
        if (msg.first_part_of_unique_id) {
            rcvd_unique_id_offset = 0;
            memset(rcvd_unique_id, 0, sizeof(rcvd_unique_id));
        } else {
            //we are only accepting first part
            return;
        }
    } else if (msg.first_part_of_unique_id) {
        // we are only accepting follow up messages
        return;
    }

    if (rcvd_unique_id_offset) {
        debug_dronecan(AP_CANManager::LOG_DEBUG, "TIME: %ld  -- Accepting Followup part! %u\n",
                     (long int)AP_HAL::millis(),
                     unsigned((now - last_alloc_msg_ms)));
    } else {
        debug_dronecan(AP_CANManager::LOG_DEBUG, "TIME: %ld  -- Accepting First part! %u\n",
                     (long int)AP_HAL::millis(),
                     unsigned((now - last_alloc_msg_ms)));
    }

    last_alloc_msg_ms = now;
    if ((rcvd_unique_id_offset + msg.unique_id.len) > 16) {
        //This request is malformed, Reset!
        rcvd_unique_id_offset = 0;
        memset(rcvd_unique_id, 0, sizeof(rcvd_unique_id));
        return;
    }

    //copy over the unique_id
    for (uint8_t i=rcvd_unique_id_offset; i<(rcvd_unique_id_offset + msg.unique_id.len); i++) {
        rcvd_unique_id[i] = msg.unique_id.data[i - rcvd_unique_id_offset];
    }
    rcvd_unique_id_offset += msg.unique_id.len;

    //send follow up message
    uavcan_protocol_dynamic_node_id_Allocation rsp {};

    /* Respond with the message containing the received unique ID so far
    or with node id if we successfully allocated one. */
    memcpy(rsp.unique_id.data, rcvd_unique_id, rcvd_unique_id_offset);
    rsp.unique_id.len = rcvd_unique_id_offset;

    if (rcvd_unique_id_offset == 16) {
        //We have received the full Unique ID, time to do allocation
        rsp.node_id = db.handleAllocation(msg.node_id, (const uint8_t*)rcvd_unique_id);
        //reset states as well        
        rcvd_unique_id_offset = 0;
        memset(rcvd_unique_id, 0, sizeof(rcvd_unique_id));
    }

    allocation_pub.broadcast(rsp, false); // never publish allocation message with CAN FD
}

// handle the allocation message. returns the new node ID.
uint8_t AP_DroneCAN_DNA_Server::Database::handleAllocation(uint8_t node_id, const uint8_t unique_id[])
{
    WITH_SEMAPHORE(sem);

    uint8_t resp_node_id = getNodeIDForUniqueID(unique_id, 16);
    if (resp_node_id == 0) {
        resp_node_id = findFreeNodeID(node_id > MAX_NODE_ID ? 0 : node_id);
        if (resp_node_id != 0) {
            addNodeIDForUniqueID(resp_node_id, unique_id, 16);
            return resp_node_id;
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "UC Node Alloc Failed!");
            return 0;
        }
    } else {
        return resp_node_id;
    }
}

//report the server state, along with failure message if any
bool AP_DroneCAN_DNA_Server::prearm_check(char* fail_msg, uint8_t fail_msg_len) const
{
    switch (server_state) {
    case HEALTHY:
        return true;
    case DUPLICATE_NODES: {
        if (_ap_dronecan.option_is_set(AP_DroneCAN::Options::DNA_IGNORE_DUPLICATE_NODE)) {
            // ignore error
            return true;
        }
        snprintf(fail_msg, fail_msg_len, "Duplicate Node %s../%d!", fault_node_name, fault_node_id);
        return false;
    }
    case NODE_STATUS_UNHEALTHY: {
        if (_ap_dronecan.option_is_set(AP_DroneCAN::Options::DNA_IGNORE_UNHEALTHY_NODE)) {
            // ignore error
            return true;
        }
        snprintf(fail_msg, fail_msg_len, "Node %d unhealthy!", fault_node_id);
        return false;
    }
    }
    // should never get; compiler should enforce all server_states are covered
    return false;
}

#endif //HAL_NUM_CAN_IFACES
