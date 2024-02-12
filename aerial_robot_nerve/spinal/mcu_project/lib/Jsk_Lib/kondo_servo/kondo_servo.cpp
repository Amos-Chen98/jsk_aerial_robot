/*
******************************************************************************
* File Name          : kondo_servo.h
* Description        : kondo servo interface
******************************************************************************
*/

#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#include "kondo_servo.h"

void KondoServo::init(UART_HandleTypeDef* huart, ros::NodeHandle* nh)
{
  huart_ = huart;
  nh_ = nh;

  nh_->subscribe(kondo_servo_control_sub_);
  nh_->advertise(joint_state_pub_);

  __HAL_UART_DISABLE_IT(huart, UART_IT_PE);
  __HAL_UART_DISABLE_IT(huart, UART_IT_ERR);
  HAL_HalfDuplex_EnableReceiver(huart_);
  HAL_UART_Receive_DMA(huart, kondo_rx_buf_, RX_BUFFER_SIZE);

  memset(kondo_rx_buf_, 0, RX_BUFFER_SIZE);
  memset(pos_rx_buf_, 0, KONDO_POSITION_RX_SIZE);

  joint_state_msg_.name_length = 4;
  joint_state_msg_.name = new char*[4];
  joint_state_msg_.name[0] = "gimbal1";
  joint_state_msg_.name[1] = "gimbal2";
  joint_state_msg_.name[2] = "gimbal3";
  joint_state_msg_.name[3] = "gimbal4";
  joint_state_msg_.position_length = 4;
  joint_state_msg_.position = new double_t[4];
}

void KondoServo::update()
{
  int servo_id = servo_name_[id_telem_] - '0';

  if (activated_[servo_id])
    receiveSendOnce(servo_id, target_position_[servo_id]);
  else
    receiveSendOnce(servo_id, 0);  // freed

  id_telem_++;
  id_telem_ %= sizeof(servo_name_) / sizeof(servo_name_[0]);

  if (id_telem_ == 1)  // the final data of the last round is received after the first data in this round is sent
    sendServoState();
}

void KondoServo::receiveSendOnce(int id, uint16_t target_position)
{
  if (is_receive_data)
  {
    memset(pos_rx_buf_, 0, KONDO_POSITION_RX_SIZE);

    uint8_t rx_ptr = 0;
    while (true)
    {
      if (!available())
        break;
      int data = readOneByte();
      pos_rx_buf_[rx_ptr] = (uint8_t)data;
      rx_ptr++;
      if (rx_ptr == KONDO_POSITION_RX_SIZE)
        break;
    }

    if (rx_ptr == KONDO_POSITION_RX_SIZE)
    {
      registerPos();
      is_receive_data = false;
    }
    else
    {
      nh_->logwarn("WARN: Kondo servo receive error");
    }
  }

  /* transmit */
  uint8_t tx_buff[KONDO_POSITION_TX_SIZE];
  uint8_t ret;

  tx_buff[0] = 0x80 + id;
  tx_buff[1] = (uint8_t)((target_position & 0x3f80) >> 7);  // higher 7 bits of 14 bits
  tx_buff[2] = (uint8_t)(target_position & 0x007f);         // lower  7 bits of 14 bits
  HAL_HalfDuplex_EnableTransmitter(huart_);
  ret = HAL_UART_Transmit(huart_, tx_buff, KONDO_POSITION_TX_SIZE, 1);

  /* receive */
  if (ret == HAL_OK)
  {
    HAL_HalfDuplex_EnableReceiver(huart_);
    is_receive_data = true;
  }
}

int KondoServo::readOneByte()
{
  /* handle RX Overrun Error */
  if (__HAL_UART_GET_FLAG(huart_, UART_FLAG_ORE))
  {
    __HAL_UART_CLEAR_FLAG(huart_, UART_CLEAR_NEF | UART_CLEAR_OREF | UART_FLAG_RXNE | UART_FLAG_ORE);
    HAL_UART_Receive_DMA(huart_, kondo_rx_buf_, RX_BUFFER_SIZE);  // restart
  }
  uint32_t dma_write_ptr = (KONDO_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart_->hdmarx)) % (KONDO_BUFFER_SIZE);

  int c = -1;
  if (kondo_rd_ptr_ != dma_write_ptr)
  {
    c = (int)kondo_rx_buf_[kondo_rd_ptr_++];
    kondo_rd_ptr_ %= KONDO_BUFFER_SIZE;
  }
  return c;
}

void KondoServo::registerPos()
{
  int id = (int)(pos_rx_buf_[0] & 0x1f);
  uint16_t current_position = (uint16_t)((0x7f & pos_rx_buf_[1]) << 7) + (uint16_t)(0x7f & pos_rx_buf_[2]);
  current_position_[id] = current_position;
}

bool KondoServo::available()
{
  uint32_t dma_write_ptr = (KONDO_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart_->hdmarx)) % (KONDO_BUFFER_SIZE);
  return (kondo_rd_ptr_ != dma_write_ptr);
}

void KondoServo::servoControlCallback(const sensor_msgs::JointState& cmd_msg)
{
  for (int i = 0; i < cmd_msg.name_length; i++)
  {
    uint8_t servo_id = cmd_msg.name[i][6] - '0';  // gimbal1
    if (servo_id >= MAX_SERVO_NUM)
      continue;

    double_t angle_rad = cmd_msg.position[i];
    if (angle_rad == 42)  // 42, the answer to the ultimate question of life, the universe, and everything
    {
      activated_[servo_id] = false;  // temporary command to free servo.
      continue;
    }

    if (angle_rad < KONDO_SERVO_ANGLE_MIN || angle_rad > KONDO_SERVO_ANGLE_MAX)
      continue;

    if (angle_rad < KONDO_SERVO_ANGLE_LIMIT_MIN)
    {
      angle_rad = KONDO_SERVO_ANGLE_LIMIT_MIN;
      nh_->logwarn("WARN: Kondo servo angle is limited to -90 degree");
    }
    else if (angle_rad > KONDO_SERVO_ANGLE_LIMIT_MAX)
    {
      angle_rad = KONDO_SERVO_ANGLE_LIMIT_MAX;
      nh_->logwarn("WARN: Kondo servo angle is limited to 90 degree");
    }

    activated_[servo_id] = true;
    target_position_[servo_id] = rad2KondoPosConv(angle_rad);
  }
}

void KondoServo::sendServoState()
{
  joint_state_msg_.header.stamp = nh_->now();
  for (uint8_t i = 1; i < 5; i++)
  {
    joint_state_msg_.position[i - 1] = kondoPos2RadConv(current_position_[i]);
  }
  joint_state_pub_.publish(&joint_state_msg_);
}

void KondoServo::setTargetPos(const std::map<uint16_t, float>& servo_map)
{
  for (auto servo : servo_map)
  {
    uint16_t id = servo.first;
    float angle = servo.second;
    uint16_t target_pos = rad2KondoPosConv(angle);

    // temporary command to free servo is angle = 100.0
    if (angle == 100.0)
    {
      activated_[id] = false;
      target_position_[id] = 7500;
      // char buf[100];
      // sprintf(buf, "servo id: %d is freed!", id);
      // nh_->l
    }
    else if (KONDO_SERVO_POSITION_MIN <= target_pos && target_pos <= KONDO_SERVO_POSITION_MAX)
    {
      activated_[id] = true;
      target_position_[id] = target_pos;
    }
  }
}

uint16_t KondoServo::rad2KondoPosConv(float angle)
{
  uint16_t kondo_pos =
      (uint16_t)((KONDO_SERVO_POSITION_MAX - KONDO_SERVO_POSITION_MIN) * (-angle - KONDO_SERVO_ANGLE_MIN) /
                     (KONDO_SERVO_ANGLE_MAX - KONDO_SERVO_ANGLE_MIN) +
                 KONDO_SERVO_POSITION_MIN);  // min-max normarization
  return kondo_pos;
}

float KondoServo::kondoPos2RadConv(int pos)
{
  float angle = -(float)((KONDO_SERVO_ANGLE_MAX - KONDO_SERVO_ANGLE_MIN) * (pos - KONDO_SERVO_POSITION_MIN) /
                             (KONDO_SERVO_POSITION_MAX - KONDO_SERVO_POSITION_MIN) +
                         KONDO_SERVO_ANGLE_MIN);  // min-max normarization
  return angle;
}