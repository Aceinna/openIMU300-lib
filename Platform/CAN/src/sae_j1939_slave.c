/** ***************************************************************************
 * @file sae_j1939_slave.c the definitions of basic functions
 * @Author Feng
 * @date   May 2017
 * @brief  Copyright (c) 2017 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 *****************************************************************************/
#ifdef SAE_J1939
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "platformAPI.h"
#include "algorithmAPI.h"
#include "UserConfiguration.h"
#include "UserMessagingCAN.h"
#include "canAPI.h"
#include "sae_j1939.h"

#define CAN_OUTPUT_INTERVAL                         15

uint32_t can_output_counter = 0;

ECU_INSTANCE *gEcu = &gEcuInst;



/** ***************************************************************************
 * @name aceinna_j1939_send_software_version() builds up software version packet
 * @brief perform sofware version feature in spec 5.1.1
 *        
 * @param [in] 
 *             
 * @retval 1 successful or 0 failure
 ******************************************************************************/
uint8_t aceinna_j1939_send_software_version(void)
{
  	msg_params_t params;
	uint8_t 	data[8];
 
  	// build up the header of software version 
    params.data_page = 0;
  	params.ext_page = 0;
    params.pkt_type = SAE_J1939_RESPONSE_PACKET;
  	params.len      = ACEINNA_SAE_J1939_VERSION_PACKET_LEN;
  	params.priority = SAE_J1939_CONTROL_PRIORITY;
  	params.PF       = SAE_J1939_PDU_FORMAT_254;
  	params.PS       = SAE_J1939_GROUP_EXTENSION_SOFTWARE_VERSION;
  
  	// build up software version payload

  	platformGetVersionBytes(data);

	return aceinna_j1939_build_msg((void *)data, &params);
  
}

/** ***************************************************************************
 * @name aceinna_j1939_send_ecu_id() builds up ecu id packet
 * @brief perform ecu id feature in spec 5.1.2
 *        
 * @param [in] 
 *             
 * @retval 1 successful or 0 failure
 ******************************************************************************/
uint8_t aceinna_j1939_send_ecu_id(void)
{
  	msg_params_t params;
  
  	// build up the header of ecu id
    params.data_page = 0;
  	params.ext_page = 0;
  	params.pkt_type = SAE_J1939_RESPONSE_PACKET;
  	params.len      = ACEINNA_SAE_J1939_ECU_PACKET_LEN;
  	params.priority = SAE_J1939_CONTROL_PRIORITY;
  	params.PF       = SAE_J1939_PDU_FORMAT_ECU;
  	params.PS       = SAE_J1939_GROUP_EXTENSION_ECU;
  
	return aceinna_j1939_build_msg((void *)&(gEcuConfigPtr->ecu_name.words), &params);
}

/** ***************************************************************************
 * @name aceinna_j1939_send_mag_align builds up mag align response packet
 *        
 * @param [in] - mag alignment status 
 *             
 * @retval 1 successful or 0 failure
 ******************************************************************************/
uint8_t aceinna_j1939_send_mag_align(uint8_t cmd, uint8_t state, real *data)
{
    MAG_ALIGN_RESPONSE resp;
  	msg_params_t params;
  
  	// build up the header of ecu id
    params.data_page   = 0;
  	params.ext_page    = 0;
  	params.pkt_type    = SAE_J1939_RESPONSE_PACKET;
  	params.len         = ACEINNA_SAE_J1939_MAG_ALIGN_LEN;
  	params.priority    = SAE_J1939_CONTROL_PRIORITY;
  	params.PF          = SAE_J1939_PDU_FORMAT_GLOBAL;
  	params.PS          = gEcuConfigPtr->mag_align_ps;
  
    resp.request       = cmd;
    resp.status        = state;
    resp.hardIronX     = (int16_t)((data[0] + 8.0) * 256 );           // +-8 Gauss, offset = -8 
    resp.hardIronY     = (int16_t)((data[1] + 8.0) * 256 );           // +-8 Gauss, offset = -8 
    resp.softIronSF    = (uint16_t)((data[2] + 0) * 1024);        // range 0 - 1, offset = 0
    resp.softIronAngle = (int16_t)((data[3] + 3.141592) * 651.8986) ; // range +- 3.141592 rad, offset = -3.141592 rad     

	return aceinna_j1939_build_msg((void *)&resp, &params);
}

/** ***************************************************************************
 * @name aceinna_j1939_send_algrst_cfgsave() builds up config save or alg reset response
 * @brief perform ecu id feature in spec 5.1.3 and 5.1.4
 *        
 * @param [in] target, ecu object
 *             alg_rst, 0--config save or 1--alg reset
 *             success, 1 sucess or 0 fails
 * @retval 1 successful or 0 failure
 ******************************************************************************/
uint8_t aceinna_j1939_send_algrst_cfgsave(ECU_ADDRESS_ENTRY *target, uint8_t alg_rst, uint8_t success)
{
  	msg_params_t params;
	uint8_t 	data[8];

  	// build up the header
    params.data_page = 0;
  	params.ext_page = 0;
  	params.pkt_type = SAE_J1939_RESPONSE_PACKET;
  	params.len      = ACEINNA_SAE_J1939_ALGO_RST_LEN;
  	params.priority = SAE_J1939_CONTROL_PRIORITY;
  	params.PF       = SAE_J1939_PDU_FORMAT_GLOBAL;
  	
	  if (alg_rst){
    	params.PS = gEcuConfigPtr->alg_reset_ps;
  	}
  	else{
    	params.PS = gEcuConfigPtr->save_cfg_ps; 
	} 
  
  	// build up payload
  	data[0] = ACEINNA_SAE_J1939_RESPONSE;
  	data[1] = target->address;
  	data[2] = success;
  
	return aceinna_j1939_build_msg((void *)data, &params);
}

/** ***************************************************************************
 * @name  aceinna_j1939_send_status_packet() status packet
 *        bit packets
 * @brief perform bit status feature in spec 5.2.1, 5.2.2 and 5.2.3
 *        
 * @param [in] built_in_type, ACEINNA_SAE_J1939_BUILTIN_HARDWARE,
 *                            ACEINNA_SAE_J1939_BUILTIN_SOFTWARE,
 *                            ACEINNA_SAE_J1939_BUILTIN_STATUS
 *             bit_fields, bit status message 
 * @retval 1 successful or 0 failure
 ******************************************************************************/
uint8_t aceinna_j1939_send_status_packet(uint8_t built_in_type, void * statusWord)
{
  	msg_params_t params;

  	// build up the header of bit status
    params.data_page = 0;
  	params.ext_page = 0;
  	params.pkt_type = SAE_J1939_RESPONSE_PACKET;
  	params.priority = SAE_J1939_CONTROL_PRIORITY;
  	params.PF       = SAE_J1939_PDU_FORMAT_GLOBAL;
    params.len      = SAE_J1939_PAYLOAD_LEN_2_BYTES;
    params.PS       = gEcuConfigPtr->status_ps;

	return aceinna_j1939_build_msg((void *)statusWord, &params);
}

/** ***************************************************************************
 * @name  aceinna_j1939_send_packet_rate() repond SET odr request
 * @brief perform ODR SET feature in spec 5.4.1
 *        
 * @param [in] target, host ecu object
 *             odr, requested by host
 * @retval 1 successful or 0 failure
 ******************************************************************************/
uint8_t aceinna_j1939_send_packet_rate(uint8_t odr)
{
  	msg_params_t params;
	uint8_t 	data[8];
  
  	// header message of response 
    params.data_page = 0;
  	params.ext_page = 0;
  	params.pkt_type = SAE_J1939_RESPONSE_PACKET;
  	params.len      = ACEINNA_SAE_J1939_PACKET_RATE_LEN;
  	params.priority = SAE_J1939_CONTROL_PRIORITY;
  	params.PF       = SAE_J1939_PDU_FORMAT_GLOBAL;
  	params.PS       = gEcuConfigPtr->packet_rate_ps;
  
  	// payload of response
  	data[0] = *(uint8_t *)gEcu->addr;
  	data[1] = odr;
  
	return aceinna_j1939_build_msg((void *)data, &params);
}


/** ***************************************************************************
 * @name  aceinna_j1939_send_packet_type() repond SET packet type request
 * @brief perform packet type SET feature in spec 5.4.2
 *        
 * @param [in] target, host ecu object
 *             type, ACEINNA_SAE_J1939_SLOPE_SENSOR_TYPE,
 *                   ACEINNA_SAE_J1939_ANGULAR_RATE_TYPE,
 *                   ACEINNA_SAE_J1939_ACCELERATOR_TYPE
 * @retval 1 successful or 0 failure
 ******************************************************************************/
uint8_t aceinna_j1939_send_packet_type(uint16_t type)
{
  	msg_params_t params;
	uint8_t 	data[8];

  // header message of packet type response
    params.data_page = 0;
  	params.ext_page = 0;
  	params.pkt_type = SAE_J1939_RESPONSE_PACKET;
  	params.len      = ACEINNA_SAE_J1939_PACKET_TYPE_LEN;
  	params.priority = SAE_J1939_CONTROL_PRIORITY;
  	params.PF       = SAE_J1939_PDU_FORMAT_GLOBAL;
  	params.PS       = gEcuConfigPtr->packet_type_ps;
  
 	 // payload of packet type response
  	data[0] = *(uint8_t *)gEcu->addr;
  	data[1] = type & 0xff;
  	data[2] = (type >> 8) & 0xff;
  
	return aceinna_j1939_build_msg((void *)data, &params);
}

/** ***************************************************************************
 * @name  aceinna_j1939_send_digital_filter() repond SET lpf request
 * @brief perform lpf SET feature in spec 5.4.3
 *        
 * @param [in] target, host ecu object
 *             accel_cutoff, lpf of xl, 2,5,10,20,50 Hz
 *             rate_cutoff, lpf of rate sensor, 2,5,10,20,50 Hz      
 * @retval 1 successful or 0 failure
 ******************************************************************************/
uint8_t aceinna_j1939_send_digital_filter(uint8_t accel_cutoff, uint8_t rate_cutoff)
{
  	msg_params_t params;
	uint8_t 	data[3];

  	// header message of lpf response
    params.data_page = 0;
  	params.ext_page = 0;
  	params.pkt_type = SAE_J1939_RESPONSE_PACKET;
  	params.len      = ACEINNA_SAE_J1939_DIGITAL_FILTER_LEN;
  	params.priority = SAE_J1939_CONTROL_PRIORITY;
  	params.PF       = SAE_J1939_PDU_FORMAT_GLOBAL;
  	params.PS       = gEcuConfigPtr->digital_filter_ps;
  
  	// payload of lpf response
  	data[0] = *(uint8_t *)gEcu->addr;
  	data[1] = accel_cutoff;
  	data[2] = rate_cutoff;

	return aceinna_j1939_build_msg((void *)data, &params);
}

/** ***************************************************************************
 * @name  aceinna_j1939_send_orientation() repond SET orientation request
 * @brief perform orientation SET feature in spec 5.4.4
 *        
 * @param [in] target, host ecu object
 *             orien, see the table of 5.4.4
 * @retval 1 successful or 0 failure
 ******************************************************************************/
uint8_t aceinna_j1939_send_orientation( uint8_t *orien)
{
  	msg_params_t params;
	uint8_t 	data[3];
  	// header message of orientation response
    params.data_page = 0;
  	params.ext_page = 0;
  	params.pkt_type = SAE_J1939_RESPONSE_PACKET;
  	params.len      = ACEINNA_SAE_J1939_ORIENTATION_LEN;
  	params.priority = SAE_J1939_CONTROL_PRIORITY;
  	params.PF       = SAE_J1939_PDU_FORMAT_GLOBAL;
  	params.PS       = gEcuConfigPtr->orientation_ps; 
  
  	// payload of orientation response
  	data[0] = *(uint8_t *)gEcu->addr;
  	data[1] = orien[0];
  	data[2] = orien[1];

	return aceinna_j1939_build_msg((void *)data, &params);
}

/** ***************************************************************************
 * @name  aceinna_j1939_send_slope_sensor() ss2 data packet 61481 (0xF029)
* @brief perform ss2 output in spec 5.6.1
 *        
 * @param [in] data, ss2 data type
 * @retval 1 successful or 0 failure
 ******************************************************************************/
uint8_t aceinna_j1939_send_slope_sensor(SLOPE_SENSOR_2 * data)
{

  	msg_params_t params;
  
  // header of ss2 packet
    params.data_page = 0;
  	params.ext_page = 0;
  	params.pkt_type = SAE_J1939_DATA_PACKET;
  	params.priority = SAE_J1939_SLOPE_PRIORITY;
  	params.PF       = SAE_J1939_PDU_FORMAT_DATA;
  	params.len      = ACEINNA_SAE_J1939_SLOPE_SENSOR2_LEN;
  	params.PS       = SAE_J1939_GROUP_EXTENSION_SLOPE_SENSOR;

	return aceinna_j1939_build_msg((void *)data, &params);
}

/** ***************************************************************************
 * @name  aceinna_j1939_send_angular_rate() angular rate data packet 61482 (0xF02A)
* @brief perform angular rate output in spec 5.6.2
 *        
 * @param [in] data, angular rate data type
 * @retval 1 successful or 0 failure
 ******************************************************************************/    
uint8_t aceinna_j1939_send_angular_rate(AUGULAR_RATE * data)
{
  	msg_params_t params;

  	// header of angular rate packet
    params.data_page = 0;
  	params.ext_page = 0;
  	params.pkt_type = SAE_J1939_DATA_PACKET;
  	params.priority = SAE_J1939_SLOPE_PRIORITY;
  	params.PF       = SAE_J1939_PDU_FORMAT_DATA;
  	params.len      = ACEINNA_SAE_J1939_ANGULAR_RATE_LEN;
  	params.PS       = SAE_J1939_GROUP_EXTENSION_ANGULAR_RATE;
	
	return aceinna_j1939_build_msg((void *)data, &params);
}


/** ***************************************************************************
 * @name  aceinna_j1939_send_acceleration() acceleration data packet 61485 (0xF02D)
* @brief perform accelaration output in spec 5.6.3
 *        
 * @param [in] data, acceleration data type
 * @retval 1 successful or 0 failure
 ******************************************************************************/    
uint8_t aceinna_j1939_send_acceleration(ACCELERATION_SENSOR * data)
{
  	msg_params_t params;
  
  // header of acceleration packet
    params.data_page = 0;
  	params.ext_page = 0;
  	params.pkt_type = SAE_J1939_DATA_PACKET;
  	params.priority = SAE_J1939_ACCELERATION_PRIORITY;
  	params.PF       = SAE_J1939_PDU_FORMAT_DATA;
  	params.len      = ACEINNA_SAE_J1939_ACCELERATION_LEN;
  	params.PS       = SAE_J1939_GROUP_EXTENSION_ACCELERATION;

	return aceinna_j1939_build_msg((void *)data, &params);
}

/** ***************************************************************************
 * @name  aceinna_j1939_send_mags() magnetometer data packet 65386 (0xFF6A)
 *        
 * @param [in] data, magnetometer data type
 * @retval 1 successful or 0 failure
 ******************************************************************************/    
uint8_t aceinna_j1939_send_mags(MAGNETIC_SENSOR * data)
{
  	msg_params_t params;
  
  // header of acceleration packet
    params.data_page = 0;
  	params.ext_page = 0;
  	params.pkt_type = SAE_J1939_DATA_PACKET;
  	params.priority = SAE_J1939_CONTROL_PRIORITY;
  	params.PF       = SAE_J1939_PDU_FORMAT_GLOBAL;
  	params.len      = ACEINNA_SAE_J1939_MAG_LEN;
  	params.PS       = SAE_J1939_GROUP_EXTENSION_MAG;

	return aceinna_j1939_build_msg((void *)data, &params);
}



/** ***************************************************************************
 * @name  aceinna_j1939_send_position() position data packet 65267 (0xfEf3)
 *        
 * @param [in] data, position data type
 * @retval 1 successful or 0 failure
 ******************************************************************************/    
uint8_t aceinna_j1939_send_GPS(GPS_DATA * data)
{
  	msg_params_t params;
  
    params.data_page = 0;
  	params.ext_page = 0;
 	params.pkt_type = SAE_J1939_DATA_PACKET;
  	params.priority = SAE_J1939_POSITION_PRIORITY;
  	params.PF       = SAE_J1939_PDU_FORMAT_254;
  	params.PS       = SAE_J1939_PDU_SPECIFIC_243;
  	params.len      = SAE_J1939_PAYLOAD_LEN_8_BYTES;
  
	return aceinna_j1939_build_msg((void *)data, &params);
}

/** ***************************************************************************
 * @name  aceinna_j1939_send_position() position data packet 129025 (1 0xF801)
 *        
 * @param [in] data, position data type
 * @retval 1 successful or 0 failure
 ******************************************************************************/    
uint8_t aceinna_j1939_send_position_rapid_update(GPS_DATA * data)
{
  	msg_params_t params;
  
    params.data_page = 1;
  	params.ext_page = 0;
 	params.pkt_type = SAE_J1939_DATA_PACKET;
  	params.priority = SAE_J1939_POSITION_PRIORITY;
  	params.PF       = SAE_J1939_PDU_FORMAT_248;
  	params.PS       = SAE_J1939_PDU_SPECIFIC_1;
  	params.len      = SAE_J1939_PAYLOAD_LEN_8_BYTES;
  
	return aceinna_j1939_build_msg((void *)data, &params);
}


/** ***************************************************************************
 * @name  aceinna_j1939_send_attitude() data packet
 * @brief perform attitude output PGN 127257 (1 0xF119)
 *        
 * @param [in] data, attitude data type
 * @retval 1 successful or 0 failure
 ******************************************************************************/    
uint8_t aceinna_j1939_send_attitude(ATTITUDE_DATA * data)
{
  	msg_params_t params;

    params.data_page = 1;
  	params.ext_page = 0;
  	params.pkt_type = SAE_J1939_DATA_PACKET;
  	params.priority = SAE_J1939_ATTITUDE_PRIORITY;
  	params.PF       = SAE_J1939_PDU_FORMAT_241;
  	params.PS       = SAE_J1939_PDU_SPECIFIC_25;
  	params.len      = SAE_J1939_PAYLOAD_LEN_8_BYTES;

	return aceinna_j1939_build_msg((void *)data, &params);
}

/** ***************************************************************************
 * @name  aceinna_j1939_send_attitude() data packet
 * @brief perform attitude output PGN 129026
 *        
 * @param [in] data, attitude data type
 * @retval 1 successful or 0 failure
 ******************************************************************************/    
uint8_t aceinna_j1939_send_course_rapid_update(COURSE_RAPID_UPDATE_DATA * data)
{
  	msg_params_t params;

    params.data_page = 1;
  	params.ext_page = 0;
  	params.pkt_type = SAE_J1939_DATA_PACKET;
  	params.priority = SAE_J1939_RUPDATE_PRIORITY;
  	params.PF       = SAE_J1939_PDU_FORMAT_248;
  	params.PS       = SAE_J1939_PDU_SPECIFIC_2;
  	params.len      = SAE_J1939_PAYLOAD_LEN_8_BYTES;

	return aceinna_j1939_build_msg((void *)data, &params);
}

/** ***************************************************************************
 * @name  aceinna_j1939_build_msg() data packet
 *        
 * @param [in] data, attitude data type
 * @retval 1 successful or 0 failure
 ******************************************************************************/    
uint8_t aceinna_j1939_build_msg(void *payload, msg_params_t *params)
{
  struct sae_j1939_tx_desc * desc;
  
  // check state machine
  if (gEcu->state < _ECU_READY) 
    return 0;
  
  // get tx desc
  if (find_tx_desc(&desc) == 0)
    return 0;
  
  // header of acceleration packet
  desc->tx_pkt_type                         = params->pkt_type;
  desc->tx_identifier.control_bits.priority = params->priority;
  desc->tx_identifier.control_bits.data_page = params->data_page;
  desc->tx_identifier.control_bits.ext_page  = params->ext_page;
  desc->tx_identifier.pdu_format            = params->PF;
  desc->tx_identifier.pdu_specific          = params->PS;
  desc->tx_payload_len                      = params->len;
  desc->tx_identifier.source                = *(uint8_t *)gEcu->addr;
  
  desc->tx_buffer.ExtId = ((desc->tx_identifier.r << 24) |
                                        (desc->tx_identifier.pdu_format << 16) |
                                        (desc->tx_identifier.pdu_specific << 8) |
                                        (desc->tx_identifier.source));
  desc->tx_buffer.IDE = CAN_ID_EXT;
  desc->tx_buffer.RTR = CAN_RTR_Data;
  desc->tx_buffer.DLC = desc->tx_payload_len;
  
  // payload of position packet
  memcpy((void *)desc->tx_buffer.Data, (void *)payload, desc->tx_payload_len);
  desc->tx_pkt_ready  = DESC_OCCUPIED;
  
  return 1;
}





/** ***************************************************************************
 * @name  is_valid_j1939_rcv() check incoming packets valid or not
* @brief a general API of received packets check
 *        
 * @param [in] rx_desc, rx descriptor
 * @retval ACEINNA_J1939_INVALID_IDENTIFIER, ACEINNA_J1939_IGNORE, ACEINNA_J1939_DATA,
 *         ACEINNA_J1939_ADDRESS_CLAIM, ACEINNA_J1939_CONFIG
 ******************************************************************************/ 
ACEINNA_J1939_PACKET_TYPE is_valid_j1939_rcv(struct sae_j1939_rx_desc *rx_desc)
{
  SAE_J1939_IDENTIFIER_FIELD *ident;
  ACEINNA_J1939_PACKET_TYPE result;
 
  // check rx desc is NULL
  if (rx_desc == NULL)
       return ACEINNA_J1939_INVALID_IDENTIFIER;

  ident = &(rx_desc->rx_identifier);
  
  // valid identifier
  if (!is_valid_sae_j1939_identifier(ident))
    return ACEINNA_J1939_INVALID_IDENTIFIER;
  
  // is aceinna data packet
  result = is_aceinna_data_packet(ident);
  if (result == ACEINNA_J1939_DATA)
     return ACEINNA_J1939_IGNORE;

  // is data packet
  result = is_algorithm_data_packet(ident);
  if (result == ACEINNA_J1939_DATA)
    goto end_j1939_rcv;

  // is address claim packet
  result = is_valid_address_claim(ident);
  if (result == ACEINNA_J1939_ADDRESS_CLAIM)
     goto end_j1939_rcv;
  
  // is SET commands
  result = is_valid_config_command(ident);
  if (result == ACEINNA_J1939_CONFIG)
    goto end_j1939_rcv;
  
  result = ACEINNA_J1939_REQUEST_PACKET;
  
end_j1939_rcv:
  
  return result;        
}

/** ***************************************************************************
 * @name  ecu_alg_reset() action while getting alg rst request
* @brief perform alg state to initialized state
 *        
 * @param [in] 
 * @retval 
 ******************************************************************************/ 
void user_alg_reset(void)
{
  InitializeAlgorithmStruct(FREQ_200_HZ);
}



/** ***************************************************************************
 * @name  process_config_set() an API of processing SET commands 
 * @brief decode identifier of incoming SET commands
 *
 * @param [in] desc, rx descriptor
 * @retval N/A
 ******************************************************************************/
void process_config_set(struct sae_j1939_rx_desc *desc)
{
  SAE_J1939_IDENTIFIER_FIELD *ident;
  uint8_t pf_val, ps_val;
  uint8_t *command;
  
  // check desc
  if (desc == NULL)
    return;
  
  // check identifier
  ident = &(desc->rx_identifier);
  if (ident == NULL)
    return;
  
  // check commands
  command = desc->rx_buffer.Data;
  if (command == NULL)
    return;
  
  pf_val = ident->pdu_format;
  ps_val = ident->pdu_specific;
  
  if (pf_val != SAE_J1939_PDU_FORMAT_GLOBAL)
    return;
  
  ProcessEcuCommands(command, ps_val, ident->source);
  
  return;
  
}

void ecu_transmit()
{
  struct sae_j1939_tx_desc *tx_desc;
  uint8_t result;

  // place outgoing message into queue
  tx_desc = gEcu->curr_tx_desc;

  while (tx_desc->next != gEcu->curr_tx_desc) 
  {
    if (tx_desc->tx_pkt_ready == DESC_IDLE) {
      tx_desc = tx_desc->next;
      continue;
    }
    result = gEcu->xmit(tx_desc);
    if (result != CAN_TxStatus_NoMailBox) {
      tx_desc->tx_pkt_ready = DESC_IDLE;
      tx_desc = tx_desc->next;
      continue;
    }
    // always end up on busy unsent descriptor
    gEcu->curr_tx_desc = tx_desc; 
    return;
  }

}

/** ***************************************************************************
 * @name  ecu_process() an API of handler processing all the incoming J1939's message
 * @brief decode the indentifier and ensure message valid 
 *
 * @param [in]
 * @retval N/A
 ******************************************************************************/
void ecu_process(void)
{
  struct sae_j1939_rx_desc *rx_desc = gEcu->curr_process_desc;
  ACEINNA_J1939_PACKET_TYPE incoming_type;
  ECU_ADDRESS_ENTRY ecu_entry;
  
  memcpy((void *)&ecu_entry.ecu_name, (void *)gEcu->name, 8);
  ecu_entry.address  = *gEcu->addr;
  ecu_entry.status   = _ECU_NORMAL;
  ecu_entry.category = _ECU_MASTER;
  
  // check rx desc
  if (rx_desc == NULL)
    return;
 
  // decode and analyze identifier
  while (rx_desc->rx_pkt_ready == DESC_OCCUPIED) {
    rx_desc->rx_identifier.control_bits.priority 	= (rx_desc->rx_buffer.ExtId >> 26) & 0x7;
    rx_desc->rx_identifier.control_bits.data_page 	= (rx_desc->rx_buffer.ExtId >> 24) & 0x1;
    rx_desc->rx_identifier.pdu_format 				= (rx_desc->rx_buffer.ExtId >> 16) & 0xff;
    rx_desc->rx_identifier.pdu_specific 			= (rx_desc->rx_buffer.ExtId >> 8) & 0xff;
    rx_desc->rx_identifier.source 					= rx_desc->rx_buffer.ExtId & 0xff;
    
    incoming_type = is_valid_j1939_rcv(rx_desc);
    if ((incoming_type == ACEINNA_J1939_IGNORE) ||
        (incoming_type == ACEINNA_J1939_INVALID_IDENTIFIER)) {
      	rx_desc->rx_pkt_ready = DESC_IDLE;
      	rx_desc = rx_desc->next;
      	continue;
    }
    
    // dispatch to message handler
    switch (incoming_type) {
    // address claim handler
    case ACEINNA_J1939_ADDRESS_CLAIM:
      process_address_claim(rx_desc);
      break;
      // request handler
    case ACEINNA_J1939_REQUEST_PACKET:
      ProcessRequest(rx_desc);
      break; 
    case ACEINNA_J1939_DATA:
      ProcessDataPackets(rx_desc);
      break; 
    default:
      // SET commands handler
      process_config_set(rx_desc);
      break;
    }
    
    rx_desc->rx_pkt_ready = DESC_IDLE;
    rx_desc = rx_desc->next;
  }
  
  gEcu->curr_process_desc =  rx_desc;

  return;
}

/** ***************************************************************************
 * @name  aceinna_j1939_transmit_isr() an API of transmitting handler
 * @brief manage transmitting queue
 *
 * @param [in]
 * @retval N/A
 ******************************************************************************/
void aceinna_j1939_transmit_isr(void)
{
    ecu_transmit();
}

/** ***************************************************************************
 * @name  aceinna_j1939_receive_isr() an API of receiving handler
 * @brief manage receiving queue
 *
 * @param [in]
 * @retval N/A
 ******************************************************************************/
void aceinna_j1939_receive_isr(void)
{
  struct sae_j1939_rx_desc *desc;
  
  desc = gEcu->curr_rx_desc;
  
  // check current rx desc
  if ((desc == NULL) || (desc->next == NULL) || (desc->rx_pkt_ready == DESC_OCCUPIED))
    return;
  
  desc->rx_pkt_ready = DESC_OCCUPIED;
  
  if (desc->next->rx_pkt_ready == DESC_IDLE)
    gEcu->curr_rx_desc = gEcu->curr_rx_desc->next;
  
  return;
}
#endif
