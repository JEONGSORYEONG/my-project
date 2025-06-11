def main(self):
        while self.is_running:
            time_start = time.time()
            try:
                image = self.image_queue.get(block=True, timeout=1)
            except queue.Empty:
                if not self.is_running:
                    break
                else:
                    continue
            
            result_image = image.copy()
    
            if self.start and self.button_state: 
                h, w = image.shape[:2]
                binary_image = self.lane_detect.get_binary(image)
                
                # 항상 라인 검출 수행 (하이브리드 제어 지원)
                result_image, lane_angle, lane_x = self.lane_detect(binary_image, image.copy())
                
                ## 개선된 이벤트 우선순위 처리 ###
                if self.turn_right_detected:
                    # 점진적 우회전 처리
                    turn_complete = self.handle_right_turn_smooth()
                    
                    # 하이브리드 제어 적용
                    twist = self.calculate_hybrid_control(lane_x, self.turn_right_progress)
                    self.mecanum_pub.publish(twist)
                    
                    if turn_complete:
                        self.turn_right_detected = False
                        self.turn_right_start_time = None
                        self.turn_right_progress = 0.0
                        self.stop_led_blink()
                        self.get_logger().info('점진적 우회전 완료')
                        
                # 횡단보도 앞에서 멈추기 (crosswalk_distance가 160~180 사이일 때만)
                elif 160 < self.crosswalk_distance < 180 and self.crosswalk_count < 2:
                    self.handle_crosswalk()
                else:
                    # 기본 주행 로직 (개선됨)
                    if lane_x >= 0 and not self.stop:
                        twist = Twist()
                        self.set_led_color(self.stop)
                        if lane_x > 150:
                            self.count_turn += 1
                            if self.count_turn > 5 and not self.start_turn:
                                self.start_turn = True
                                self.set_led_right_async()
                                self.count_turn = 0
                                self.start_turn_time_stamp = time.time()
                            if self.machine_type != 'MentorPi_Acker':
                                twist.angular.z = -0.45
                            else:
                                twist.angular.z = twist.linear.x * math.tan(-0.5061) / 0.145
                        else:
                            self.count_turn = 0
                            if time.time() - self.start_turn_time_stamp > 2 and self.start_turn:
                                self.start_turn = False
                            if not self.start_turn:
                                self.pid.SetPoint = 130
                                self.pid.update(lane_x)
                                # Anti-windup PID 적용
                                if self.pid_anti_windup_enabled:
                                    pid_output = common.set_range(self.pid.output, -0.15, 0.15)
                                else:
                                    pid_output = common.set_range(self.pid.output, -0.1, 0.1)
                                
                                if self.machine_type != 'MentorPi_Acker':
                                    twist.angular.z = pid_output
                                else:
                                    twist.angular.z = twist.linear.x * math.tan(pid_output) / 0.145
                            else:
                                if self.machine_type == 'MentorPi_Acker':
                                    twist.angular.z = 0.15 * math.tan(-0.5061) / 0.145
                        
                        twist.linear.x = self.normal_speed
                        self.mecanum_pub.publish(twist)
                        # 마지막 유효 라인 위치 저장
                        self.last_lane_x = lane_x
                    else:
                        # PID 적분기 초기화 (Anti-windup)
                        if self.pid_anti_windup_enabled:
                            self.pid.clear()
