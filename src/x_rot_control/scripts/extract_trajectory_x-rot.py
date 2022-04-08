   
        # trajectory
        if msg.arbitration_id == 0x190:

            data = list(msg.data)
        
            # X position                                    
            n0 = data[0]
            n1 = data[1]
            n2 = data[2]

            if(n0 & 0b10000000):
                num = (0xff << 24) | (n0 << 16) | (n1 << 8) | n2
                num = 0xffffffff - num + 0b1
                pos_x = - np.float32(num)/100
            else:
                num = (0x00 << 24) | (n0 << 16) | (n1 << 8) | n2
                pos_x = np.float32(num)/100

            # Y position                    
            n0 = data[3]
            n1 = data[4]
            n2 = data[5]

            if(n0 & 0b10000000):
                num = (0xff << 24) | (n0 << 16) | (n1 << 8) | n2
                num = 0xffffffff - num + 0b1
                pos_y = -np.float32(num)/100
            else:
                num = (0x00 << 24) | (n0 << 16) | (n1 << 8) | n2
                pos_y = np.float32(num)/100
            
            # angle
            n0 = data[6]
            n1 = data[7]
            if(n0 & 0b10000000):
                num = (0xff << 24) | (0xff << 16) | (n0 << 8) | n1
                num = 0xffffffff - num + 0b1
                angle = - np.float32(num)/100
            else:
                num = (0x00 << 24) | (0x00 << 16) | (n0 << 8) | n1
                angle = np.float32(num)/100

            quat = quaternion_from_euler(0, 0, angle)     

        if msg.arbitration_id == 0x210:

            data = list(msg.data)
        
            # X position                                    
            n0 = data[0]
            n1 = data[1]
            n2 = data[2]

            if(n0 & 0b10000000):
                num = (0xff << 24) | (n0 << 16) | (n1 << 8) | n2
                num = 0xffffffff - num + 0b1
                pos_x = - np.float32(num)/100
            else:
                num = (0x00 << 24) | (n0 << 16) | (n1 << 8) | n2
                pos_x = np.float32(num)/100

            # Y position                    
            n0 = data[3]
            n1 = data[4]
            n2 = data[5]

            if(n0 & 0b10000000):
                num = (0xff << 24) | (n0 << 16) | (n1 << 8) | n2
                num = 0xffffffff - num + 0b1
                pos_y = -np.float32(num)/100
            else:
                num = (0x00 << 24) | (n0 << 16) | (n1 << 8) | n2
                pos_y = np.float32(num)/100
            
            # angle
            n0 = data[6]
            n1 = data[7]
            if(n0 & 0b10000000):
                num = (0xff << 24) | (0xff << 16) | (n0 << 8) | n1
                num = 0xffffffff - num + 0b1
                angle = - np.float32(num)/100
            else:
                num = (0x00 << 24) | (0x00 << 16) | (n0 << 8) | n1
                angle = np.float32(num)/100

            quat = quaternion_from_euler(0, 0, angle)
            
        if msg.arbitration_id == 0x290:

            data = list(msg.data)
        
            # X position                                    
            n0 = data[0]
            n1 = data[1]
            n2 = data[2]

            if(n0 & 0b10000000):
                num = (0xff << 24) | (n0 << 16) | (n1 << 8) | n2
                num = 0xffffffff - num + 0b1
                pos_x = - np.float32(num)/100
            else:
                num = (0x00 << 24) | (n0 << 16) | (n1 << 8) | n2
                pos_x = np.float32(num)/100

            # Y position                    
            n0 = data[3]
            n1 = data[4]
            n2 = data[5]

            if(n0 & 0b10000000):
                num = (0xff << 24) | (n0 << 16) | (n1 << 8) | n2
                num = 0xffffffff - num + 0b1
                pos_y = -np.float32(num)/100
            else:
                num = (0x00 << 24) | (n0 << 16) | (n1 << 8) | n2
                pos_y = np.float32(num)/100
            
            # angle
            n0 = data[6]
            n1 = data[7]
            if(n0 & 0b10000000):
                num = (0xff << 24) | (0xff << 16) | (n0 << 8) | n1
                num = 0xffffffff - num + 0b1
                angle = - np.float32(num)/100
            else:
                num = (0x00 << 24) | (0x00 << 16) | (n0 << 8) | n1
                angle = np.float32(num)/100

            quat = quaternion_from_euler(0, 0, angle)
            
        if msg.arbitration_id == 0x310:

            data = list(msg.data)
        
            # X position                                    
            n0 = data[0]
            n1 = data[1]
            n2 = data[2]

            if(n0 & 0b10000000):
                num = (0xff << 24) | (n0 << 16) | (n1 << 8) | n2
                num = 0xffffffff - num + 0b1
                pos_x = - np.float32(num)/100
            else:
                num = (0x00 << 24) | (n0 << 16) | (n1 << 8) | n2
                pos_x = np.float32(num)/100

            # Y position                    
            n0 = data[3]
            n1 = data[4]
            n2 = data[5]

            if(n0 & 0b10000000):
                num = (0xff << 24) | (n0 << 16) | (n1 << 8) | n2
                num = 0xffffffff - num + 0b1
                pos_y = -np.float32(num)/100
            else:
                num = (0x00 << 24) | (n0 << 16) | (n1 << 8) | n2
                pos_y = np.float32(num)/100
            
            # angle
            n0 = data[6]
            n1 = data[7]
            if(n0 & 0b10000000):
                num = (0xff << 24) | (0xff << 16) | (n0 << 8) | n1
                num = 0xffffffff - num + 0b1
                angle = - np.float32(num)/100
            else:
                num = (0x00 << 24) | (0x00 << 16) | (n0 << 8) | n1
                angle = np.float32(num)/100

            quat = quaternion_from_euler(0, 0, angle)
        
        if msg.arbitration_id == 0x390:

            data = list(msg.data)
        
            # X position                                    
            n0 = data[0]
            n1 = data[1]
            n2 = data[2]

            if(n0 & 0b10000000):
                num = (0xff << 24) | (n0 << 16) | (n1 << 8) | n2
                num = 0xffffffff - num + 0b1
                pos_x = - np.float32(num)/100
            else:
                num = (0x00 << 24) | (n0 << 16) | (n1 << 8) | n2
                pos_x = np.float32(num)/100

            # Y position                    
            n0 = data[3]
            n1 = data[4]
            n2 = data[5]

            if(n0 & 0b10000000):
                num = (0xff << 24) | (n0 << 16) | (n1 << 8) | n2
                num = 0xffffffff - num + 0b1
                pos_y = -np.float32(num)/100
            else:
                num = (0x00 << 24) | (n0 << 16) | (n1 << 8) | n2
                pos_y = np.float32(num)/100
            
            # angle
            n0 = data[6]
            n1 = data[7]
            if(n0 & 0b10000000):
                num = (0xff << 24) | (0xff << 16) | (n0 << 8) | n1
                num = 0xffffffff - num + 0b1
                angle = - np.float32(num)/100
            else:
                num = (0x00 << 24) | (0x00 << 16) | (n0 << 8) | n1
                angle = np.float32(num)/100

            quat = quaternion_from_euler(0, 0, angle)
        
        if msg.arbitration_id == 0x410:

            data = list(msg.data)
        
            # X position                                    
            n0 = data[0]
            n1 = data[1]
            n2 = data[2]

            if(n0 & 0b10000000):
                num = (0xff << 24) | (n0 << 16) | (n1 << 8) | n2
                num = 0xffffffff - num + 0b1
                pos_x = - np.float32(num)/100
            else:
                num = (0x00 << 24) | (n0 << 16) | (n1 << 8) | n2
                pos_x = np.float32(num)/100

            # Y position                    
            n0 = data[3]
            n1 = data[4]
            n2 = data[5]

            if(n0 & 0b10000000):
                num = (0xff << 24) | (n0 << 16) | (n1 << 8) | n2
                num = 0xffffffff - num + 0b1
                pos_y = -np.float32(num)/100
            else:
                num = (0x00 << 24) | (n0 << 16) | (n1 << 8) | n2
                pos_y = np.float32(num)/100
            
            # angle
            n0 = data[6]
            n1 = data[7]
            if(n0 & 0b10000000):
                num = (0xff << 24) | (0xff << 16) | (n0 << 8) | n1
                num = 0xffffffff - num + 0b1
                angle = - np.float32(num)/100
            else:
                num = (0x00 << 24) | (0x00 << 16) | (n0 << 8) | n1
                angle = np.float32(num)/100

            quat = quaternion_from_euler(0, 0, angle)
                        
        if msg.arbitration_id == 0x490:

            data = list(msg.data)
        
            # X position                                    
            n0 = data[0]
            n1 = data[1]
            n2 = data[2]

            if(n0 & 0b10000000):
                num = (0xff << 24) | (n0 << 16) | (n1 << 8) | n2
                num = 0xffffffff - num + 0b1
                pos_x = - np.float32(num)/100
            else:
                num = (0x00 << 24) | (n0 << 16) | (n1 << 8) | n2
                pos_x = np.float32(num)/100

            # Y position                    
            n0 = data[3]
            n1 = data[4]
            n2 = data[5]

            if(n0 & 0b10000000):
                num = (0xff << 24) | (n0 << 16) | (n1 << 8) | n2
                num = 0xffffffff - num + 0b1
                pos_y = -np.float32(num)/100
            else:
                num = (0x00 << 24) | (n0 << 16) | (n1 << 8) | n2
                pos_y = np.float32(num)/100
            
            # angle
            n0 = data[6]
            n1 = data[7]
            if(n0 & 0b10000000):
                num = (0xff << 24) | (0xff << 16) | (n0 << 8) | n1
                num = 0xffffffff - num + 0b1
                angle = - np.float32(num)/100
            else:
                num = (0x00 << 24) | (0x00 << 16) | (n0 << 8) | n1
                angle = np.float32(num)/100
          
            quat = quaternion_from_euler(0, 0, angle)
            
        if msg.arbitration_id == 0x510:

            data = list(msg.data)
        
            # X position                                    
            n0 = data[0]
            n1 = data[1]
            n2 = data[2]

            if(n0 & 0b10000000):
                num = (0xff << 24) | (n0 << 16) | (n1 << 8) | n2
                num = 0xffffffff - num + 0b1
                pos_x = - np.float32(num)/100
            else:
                num = (0x00 << 24) | (n0 << 16) | (n1 << 8) | n2
                pos_x = np.float32(num)/100

            # Y position                    
            n0 = data[3]
            n1 = data[4]
            n2 = data[5]

            if(n0 & 0b10000000):
                num = (0xff << 24) | (n0 << 16) | (n1 << 8) | n2
                num = 0xffffffff - num + 0b1
                pos_y = -np.float32(num)/100
            else:
                num = (0x00 << 24) | (n0 << 16) | (n1 << 8) | n2
                pos_y = np.float32(num)/100
            
            # angle
            n0 = data[6]
            n1 = data[7]
            if(n0 & 0b10000000):
                num = (0xff << 24) | (0xff << 16) | (n0 << 8) | n1
                num = 0xffffffff - num + 0b1
                angle = - np.float32(num)/100
            else:
                num = (0x00 << 24) | (0x00 << 16) | (n0 << 8) | n1
                angle = np.float32(num)/100
         
            quat = quaternion_from_euler(0, 0, angle)
            
        if msg.arbitration_id == 0x590:

            data = list(msg.data)
        
            # X position                                    
            n0 = data[0]
            n1 = data[1]
            n2 = data[2]

            if(n0 & 0b10000000):
                num = (0xff << 24) | (n0 << 16) | (n1 << 8) | n2
                num = 0xffffffff - num + 0b1
                pos_x = - np.float32(num)/100
            else:
                num = (0x00 << 24) | (n0 << 16) | (n1 << 8) | n2
                pos_x = np.float32(num)/100

            # Y position                    
            n0 = data[3]
            n1 = data[4]
            n2 = data[5]

            if(n0 & 0b10000000):
                num = (0xff << 24) | (n0 << 16) | (n1 << 8) | n2
                num = 0xffffffff - num + 0b1
                pos_y = -np.float32(num)/100
            else:
                num = (0x00 << 24) | (n0 << 16) | (n1 << 8) | n2
                pos_y = np.float32(num)/100
            
            # angle
            n0 = data[6]
            n1 = data[7]
            if(n0 & 0b10000000):
                num = (0xff << 24) | (0xff << 16) | (n0 << 8) | n1
                num = 0xffffffff - num + 0b1
                angle = - np.float32(num)/100
            else:
                num = (0x00 << 24) | (0x00 << 16) | (n0 << 8) | n1
                angle = np.float32(num)/100

            quat = quaternion_from_euler(0, 0, angle)
        
        if msg.arbitration_id == 0x610:

            data = list(msg.data)
        
            # X position                                    
            n0 = data[0]
            n1 = data[1]
            n2 = data[2]

            if(n0 & 0b10000000):
                num = (0xff << 24) | (n0 << 16) | (n1 << 8) | n2
                num = 0xffffffff - num + 0b1
                pos_x = - np.float32(num)/100
            else:
                num = (0x00 << 24) | (n0 << 16) | (n1 << 8) | n2
                pos_x = np.float32(num)/100

            # Y position                    
            n0 = data[3]
            n1 = data[4]
            n2 = data[5]

            if(n0 & 0b10000000):
                num = (0xff << 24) | (n0 << 16) | (n1 << 8) | n2
                num = 0xffffffff - num + 0b1
                pos_y = -np.float32(num)/100
            else:
                num = (0x00 << 24) | (n0 << 16) | (n1 << 8) | n2
                pos_y = np.float32(num)/100
            
            # angle
            n0 = data[6]
            n1 = data[7]
            if(n0 & 0b10000000):
                num = (0xff << 24) | (0xff << 16) | (n0 << 8) | n1
                num = 0xffffffff - num + 0b1
                angle = - np.float32(num)/100
            else:
                num = (0x00 << 24) | (0x00 << 16) | (n0 << 8) | n1
                angle = np.float32(num)/100

            quat = quaternion_from_euler(0, 0, angle)
            

