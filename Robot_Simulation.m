function Robot_Simulation()
  close all
  clc
  
  %parameters
  dt=0.1;%time step
  Hb=120; %base height
  Hp=190; %link 2 length
  lb2=270; %link 3 length
  Ln=80; %link 4 length

  %home position
  home_pos=[-270 0 230];
  %loading bay position
  load_pos=[0 322.5 15];
  %end effector positions
  pos=zeros(326,3);
  pos(1,:)=home_pos; %initial position
  pos(326,:)=home_pos; %final position
  %block positions
  for i=1:54
      layer=ceil(i/3); %#layer
      num=i-3*(layer-1); %block 1,2,3
      if mod(layer,2)==1
          b_x=-372.5+num*25; %x_pos
          b_y=0;
      else
          b_x=-322.5;
          b_y=-50+num*25; %y_pos
      end
      b_z=layer*15; %z_pos
      
      %Loading Bay- Via1- Via2- Block -Via2 -Via1 -Loading......
      pos(i*6-4,:)=load_pos; %load position
      pos(i*6-3,:)=[0 322.5 b_z+20];       %via point 1
      pos(i*6-2,:)=[b_x b_y b_z+20];       %via point 2
      pos(i*6-1,:)=[b_x b_y b_z]; %block position
      pos(i*6,:)=[b_x b_y b_z+20];       %via point 2
      pos(i*6+1,:)=[0 322.5 b_z+20];       %via point 1
  end

  
  %animation
  myVideo = VideoWriter('robotics_animation.mp4');
  open(myVideo);
  dir=0; %moving block rotation angle
  count = 0; %count of #block to be placed
  placed_pos=zeros(54,3);
  %trajectory generation
  for i=1:length(pos)-1
      x0=pos(i,1);
      y0=pos(i,2);
      z0=pos(i,3);
      xf=pos(i+1,1);
      yf=pos(i+1,2);
      zf=pos(i+1,3);
      if (i~=1 || i~=length(pos)-1) && (mod(i,6)==1 ||mod(i,6)==2 ||mod(i,6)==4 || mod(i,6)== 5)
          %If motion between Loading bay & Via Point1 or between Block &Via Point2
          Tf=1; %Time spent for trajectory
      else 
          %motion between Via point 1 and Via Point 2
          Tf=3;
      end
      %generate the trajectory
      [xt, yt, zt]=traj_generation(x0,y0,z0,xf,yf,zf,Tf);
      
      if mod(i,6)==5
          %Once a block is placed
          dir=0; %reset moving block angle
          count=count+1; %block placed count +1
          placed_pos(count,:)=[x0 y0 z0];%placed block positions
      end
        
      for t=0:dt:Tf
        %evaluate the numerical position of end effector at time t  
        xe=polyval(flipud(xt),t);
        ye=polyval(flipud(yt),t);
        ze=polyval(flipud(zt),t);
        r0e=[xe ye ze];%end effector position
        
        zw=ze+Ln;
        r0w=[xe ye zw];%wrist position
        
        origin=[0 0 0];%oringin
        
        z2=Hb;%base height
        r01=[0 0 z2];%joint 1 2 position
        
        %inverse kinematics: find q1 and q2
        [q1, q2]=inverse_kinematics(xe,ye,zw,z2,Hp,lb2);
        
        %find position of joint 3
        %rotation matrix
        R01 = [cosd(q1) -sind(q1) 0; sind(q1) cosd(q1) 0; 0 0 1];
        R12 = [sind(q2) cosd(q2) 0; 0 0 1; cosd(q2) -sind(q2) 0];
        r12_1 = [0; 0; Hb]; %r12 in frame {1}
        r23_2 = [Hp; 0; 0]; %r23 in frame {2}
        r03=R01*r12_1+R01*R12*r23_2; %r03 in frame {0}
        
        %draw the arm
        v=[origin;r01;r03';r0w;r0e];
        plot3(v(:,1),v(:,2),v(:,3),'b');
        
        %draw the moving block
        if mod(i,6)==2 ||mod(i,6)==3 ||mod(i,6)==4
            %motion from Loading Bay to Block
            if mod(ceil((count+1)/3),2)==1 %odd #layer
                %no rotation
                draw_block(xe,ye,ze,0); 
            else %even #layer
                %gradually rotate block by 90 degrees
                draw_block(xe,ye,ze,dir);
                if dir<90 && mod(i,6)>2
                    %rotate the block when between Via1 and Via2
                    step_size=90*dt/Tf; %minimum rotation at each time step
                    dir=dir+step_size*3;
                end
            end
        end
        %draw loading bay
        draw_block(0,322.5,15,0); 
        
        %draw placed block
        for j=1:count
            x_placed=placed_pos(j,1);
            y_placed=placed_pos(j,2);
            z_placed=placed_pos(j,3);
            if mod(ceil(j/3),2)==1
                %odd layer no roation
                draw_block(x_placed,y_placed,z_placed,0);
            else
                %even layer rotate block by 90 degrees
                draw_block(x_placed,y_placed,z_placed,90);
            end
        end
        
        hold off;
        %axis settings
        axis([-400 200 -150 380 0 400]); %axis([x_range y_range z_range])
        xlabel('x');
        ylabel('y');
        zlabel('z');
        grid on;
        
        %view(0,0);  %uncomment for +y side view
        %view(90,0);  %uncomment for -x side view
        %view(0,90);  %uncomment for top view
        F=getframe;
        writeVideo(myVideo, F);
      end
  end
  close(myVideo);

end

function draw_block(x,y,z,dir)
        R=[cosd(dir) -sind(dir) 0; sind(dir) cosd(dir) 0; 0 0 1];
        hold on;
        %block dimensions
        block_length = 75;
        block_width = 25;
        block_height = 15;
        %vertices positions
        b1=(R*[block_width/2 block_length/2 0]')'+[x y z];
        b2=(R*[-block_width/2 block_length/2 0]')'+[x y z];
        b3=(R*[-block_width/2 -block_length/2 0]')'+[x y z];
        b4=(R*[block_width/2 -block_length/2 0]')'+[x y z];
        b5=(R*[block_width/2 block_length/2 -block_height]')'+[x y z];
        b6=(R*[-block_width/2 block_length/2 -block_height]')'+[x y z];
        b7=(R*[-block_width/2 -block_length/2 -block_height]')'+[x y z];
        b8=(R*[block_width/2 -block_length/2 -block_height]')'+[x y z];
        %Block Position Vectors
        b_up=[b1;b2;b3;b4;b1]; %upper rectangle
        b_down=[b5;b6;b7;b8;b5]; %lower rectangle
        b_v1 = [b1;b5];
        b_v2 = [b2;b6];
        b_v3 = [b3;b7];
        b_v4 = [b4;b8];
        %Draw a block
        plot3(b_up(:,1),b_up(:,2),b_up(:,3),'r')
        plot3(b_down(:,1),b_down(:,2),b_down(:,3),'r')
        plot3(b_v1(:,1),b_v1(:,2),b_v1(:,3),'r')
        plot3(b_v2(:,1),b_v2(:,2),b_v2(:,3),'r')
        plot3(b_v3(:,1),b_v3(:,2),b_v3(:,3),'r')
        plot3(b_v4(:,1),b_v4(:,2),b_v4(:,3),'r')        
end

function [xt, yt, zt]=traj_generation(x0,y0,z0,xf,yf,zf,Tf)
      %trajectory for the end effector;
      A = [1 0 0 0; 0 1 0 0; 1 Tf Tf^2 Tf^3; 0 1 2*Tf 3*Tf^2];
      xt = A\[x0; 0; xf; 0];
      yt = A\[y0; 0; yf; 0];
      zt = A\[z0; 0; zf; 0];
end

function [q1,q2]=inverse_kinematics(xe,ye,zw,z2,Hp,lb2)
        q1 = atand(ye/abs(xe));
        L=(xe^2+ye^2+(zw-z2)^2)^0.5;
        alpha=acosd((Hp^2+lb2^2-L^2)/(2*Hp*lb2));
        if alpha<0
            alpha=alpha+180;
        end
        beta=atand((xe^2+ye^2)^0.5/abs(z2-zw));
        gamma=asind(lb2*sind(alpha)/L);
        if zw>z2
            q2=gamma-beta;
        else
            q2=beta+gamma-180;
        end

end

