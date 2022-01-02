clear all;
close all;

V1=1;
V2=0;
cpt=1;
cpt2=1;
cpt3=1;
sum=0;
X_tmp=[];
Y_tmp=[];
X_tip=0;
Y_tip=0;
theta_tip=0;
theta_tmp=[];

%Plot the tip of the probe
figure(1)
axis([-3 100 -50 50]);
hold on;

init_x=[-3,0];
init_y=[0,0];
plot(init_x, init_y,'b','linewidth', 5)
sim('probe.slx');
Pos=yout.getElement('Pos');
theta=Pos.Values.Data(:,3);
X_pos=Pos.Values.Data(:,1)+X_tip;
Y_pos=Pos.Values.Data(:,2)+Y_tip;
plot(X_pos,Y_pos,'r--')

%Simulate some organs

center1=[50,-10];
center2=[70,30];
radius=15;
radius2=10;
viscircles(center1, radius, 'Color','r');
viscircles(center2, radius2, 'Color','r');


while true
key_pressed=waitforbuttonpress;
val=get(gcf,'currentCharacter')

%quit the code!
if val=='q'
    close all;
    break;  
    
    
%changing the delta and plot the direction
elseif val=='a'
    
    
    hold off;
    plot(init_x, init_y,'b','linewidth', 5)
    hold on;
    viscircles(center1, radius, 'Color','r');
    viscircles(center2, radius2, 'Color','r');
    
    if size(X_tmp)==0 
        
        V2=V2+0.1;
        sim('probe.slx');
        Pos=yout.getElement('Pos');
        theta=Pos.Values.Data(:,3);
        X_pos=Pos.Values.Data(:,1)+X_tip;
        Y_pos=Pos.Values.Data(:,2)+Y_tip;
        plot(X_pos,Y_pos,'r--')

        hold off;
        axis([-3 100 -50 50]); 
    
    else
        
        if (sum-cpt+1)==0
            
           
            prev_theta=theta_tmp(1);
            disp(prev_theta)
            Rot=[cos(theta_tip), -sin(theta_tip); sin(theta_tip), cos(theta_tip)];
            
            Rot2=[cos(prev_theta), -sin(prev_theta); sin(prev_theta), cos(prev_theta)];

            V2=V2+0.1;

            plot(X_tmp(1:sum),Y_tmp(1:sum), 'b-', 'linewidth',5)
            sim('probe.slx');
            Pos=yout.getElement('Pos');
            theta=Pos.Values.Data(:,3);
            X_1=Pos.Values.Data(:,1);
            Y_1=Pos.Values.Data(:,2);
            cpt2=cpt-1;
            A = Rot2*Rot*[X_1'; Y_1'];
            X_pos=(A(1,:)+X_tip)';
            Y_pos=(A(2,:)+Y_tip)';


            plot(X_pos,Y_pos,'r--')

            hold off;
            axis([-3 100 -50 50]);
            
        elseif (cpt3==1)
            
            cpt3=0;
            disp(theta_tmp)
            prev_theta=prev_theta+theta_tmp(sum-cpt+1);
            disp(prev_theta)
            Rot=[cos(theta_tip), -sin(theta_tip); sin(theta_tip), cos(theta_tip)];
            Rot2=[cos(prev_theta), -sin(prev_theta); sin(prev_theta), cos(prev_theta)];

            V2=V2+0.1;

            plot(X_tmp(1:sum),Y_tmp(1:sum), 'b-', 'linewidth',5)
            sim('probe.slx');
            Pos=yout.getElement('Pos');
            theta=Pos.Values.Data(:,3);
            X_1=Pos.Values.Data(:,1);
            Y_1=Pos.Values.Data(:,2);
            cpt2=cpt-1;
            A = Rot2*Rot*[X_1'; Y_1'];
            X_pos=(A(1,:)+X_tip)';
            Y_pos=(A(2,:)+Y_tip)';


            plot(X_pos,Y_pos,'r--')

            hold off;
            axis([-3 100 -50 50]);
            
        else
            
            Rot=[cos(theta_tip), -sin(theta_tip); sin(theta_tip), cos(theta_tip)];
            Rot2=[cos(prev_theta), -sin(prev_theta); sin(prev_theta), cos(prev_theta)];

            V2=V2+0.1;

            plot(X_tmp(1:sum),Y_tmp(1:sum), 'b-', 'linewidth',5)
            sim('probe.slx');
            Pos=yout.getElement('Pos');
            theta=Pos.Values.Data(:,3);
            X_1=Pos.Values.Data(:,1);
            Y_1=Pos.Values.Data(:,2);
            cpt2=cpt-1;
            A = Rot2*Rot*[X_1'; Y_1'];
            X_pos=(A(1,:)+X_tip)';
            Y_pos=(A(2,:)+Y_tip)';


            plot(X_pos,Y_pos,'r--')

            hold off;
            axis([-3 100 -50 50]);
        end


   end
    

 elseif val=='d'
     
    
    Rot=[cos(theta_tip), -sin(theta_tip); sin(theta_tip), cos(theta_tip)];
    hold off;
    plot(init_x, init_y, 'b','linewidth', 5)
    hold on;
    viscircles(center1, radius, 'Color','r')
    viscircles(center2, radius2, 'Color','r')
    
    if size(X_tmp)==0
        
        V2=V2-0.1; 
        
        sim('probe.slx');
        Pos=yout.getElement('Pos');
        theta=Pos.Values.Data(:,3);
        X_pos=Pos.Values.Data(:,1)+X_tip;
        Y_pos=Pos.Values.Data(:,2)+Y_tip;
        plot(X_pos,Y_pos,'r--')
    
        hold off;
        axis([-3 100 -50 50]); 
    
    
         
    else
        if (sum-cpt+1)==0
            
           
            prev_theta=theta_tmp(1);
           
            Rot=[cos(theta_tip), -sin(theta_tip); sin(theta_tip), cos(theta_tip)];
            
            Rot2=[cos(prev_theta), -sin(prev_theta); sin(prev_theta), cos(prev_theta)];

            V2=V2-0.1;

            plot(X_tmp(1:sum),Y_tmp(1:sum), 'b-', 'linewidth',5)
            sim('probe.slx');
            Pos=yout.getElement('Pos');
            theta=Pos.Values.Data(:,3);
            X_1=Pos.Values.Data(:,1);
            Y_1=Pos.Values.Data(:,2);
            cpt2=cpt-1;
            A = Rot2*Rot*[X_1'; Y_1'];
            X_pos=(A(1,:)+X_tip)';
            Y_pos=(A(2,:)+Y_tip)';


            plot(X_pos,Y_pos,'r--')

            hold off;
            axis([-3 100 -50 50]);
            
        elseif (cpt3==1)
            
            cpt3=0;
           
            prev_theta=prev_theta+theta_tmp(sum-cpt+1);
          
            Rot=[cos(theta_tip), -sin(theta_tip); sin(theta_tip), cos(theta_tip)];
            Rot2=[cos(prev_theta), -sin(prev_theta); sin(prev_theta), cos(prev_theta)];

            V2=V2-0.1;

            plot(X_tmp(1:sum),Y_tmp(1:sum), 'b-', 'linewidth',5)
            sim('probe.slx');
            Pos=yout.getElement('Pos');
            theta=Pos.Values.Data(:,3);
            X_1=Pos.Values.Data(:,1);
            Y_1=Pos.Values.Data(:,2);
            cpt2=cpt-1;
            A = Rot2*Rot*[X_1'; Y_1'];
            X_pos=(A(1,:)+X_tip)';
            Y_pos=(A(2,:)+Y_tip)';


            plot(X_pos,Y_pos,'r--')

            hold off;
            axis([-3 100 -50 50]);
            
        else 
            
            Rot=[cos(theta_tip), -sin(theta_tip); sin(theta_tip), cos(theta_tip)];
            Rot2=[cos(prev_theta), -sin(prev_theta); sin(prev_theta), cos(prev_theta)];

            V2=V2-0.1;

            plot(X_tmp(1:sum),Y_tmp(1:sum), 'b-', 'linewidth',5)
            sim('probe.slx');
            Pos=yout.getElement('Pos');
            theta=Pos.Values.Data(:,3);
            X_1=Pos.Values.Data(:,1);
            Y_1=Pos.Values.Data(:,2);
            cpt2=cpt-1;
            A = Rot2*Rot*[X_1'; Y_1'];
            X_pos=(A(1,:)+X_tip)';
            Y_pos=(A(2,:)+Y_tip)';


            plot(X_pos,Y_pos,'r--')

            hold off;
            axis([-3 100 -50 50]);    
        end
        

    end
    

%Make the probe moove forward
elseif val=='z'
      viscircles(center1, radius, 'Color','r');
      viscircles(center2, radius2, 'Color','r');
    
      if cpt==1
        axis([-3 100 -50 50]);
        hold on;
        plot(X_pos,Y_pos,'r--')
        plot(init_x, init_y,'b','linewidth', 5)
        cpt=cpt+1;
        sum=sum+1;
        plot([X_pos(cpt-1),X_pos(cpt)],[Y_pos(cpt-1),Y_pos(cpt)], 'b-', 'linewidth',5)
        X_tip=X_pos(cpt);
        Y_tip=Y_pos(cpt);
        theta_tip=theta(cpt);
        X_tmp=[X_tmp ; X_tip];
        Y_tmp=[Y_tmp;Y_tip];
        theta_tmp=[theta_tmp;theta_tip];
        cpt3=1;
        
        
          
      elseif not(theta(cpt2)==theta_tip) & not(cpt2==1)
        
        
        axis([-3 100 -50 50]);
        hold on;
        cpt=1;
        sum=sum+1;
        cpt2=1;
        
        plot(X_pos,Y_pos,'r--')
        plot(init_x, init_y,'b','linewidth', 5)
        cpt=cpt+1;
        plot([X_pos(cpt-1),X_pos(cpt)],[Y_pos(cpt-1),Y_pos(cpt)], 'b-', 'linewidth',5)
        X_tip=X_pos(cpt);
        Y_tip=Y_pos(cpt);
        theta_tip=theta(cpt);
        X_tmp=[X_tmp ; X_tip];
        Y_tmp=[Y_tmp;Y_tip];
        theta_tmp=[theta_tmp;theta_tip];
        cpt3=1;
      
      else
          
        axis([-3 100 -50 50]);
        hold on;
      
        plot(X_pos,Y_pos,'r--')
        plot(init_x, init_y,'b','linewidth', 5)
        cpt=cpt+1;
        sum=sum+1;
        plot([X_pos(cpt-1),X_pos(cpt)],[Y_pos(cpt-1),Y_pos(cpt)], 'b-', 'linewidth',5)
        X_tip=X_pos(cpt);
        Y_tip=Y_pos(cpt);
        theta_tip=theta(cpt);
        X_tmp=[X_tmp ; X_tip];
        Y_tmp=[Y_tmp;Y_tip];
        theta_tmp=[theta_tmp;theta_tip];
        cpt3=1;
        
          
      end
    
    
    
%Make the probe moove backward
elseif val=='s'
    viscircles(center1, radius, 'Color','r');
    viscircles(center2, radius2, 'Color','r');
    
    hold off;
    plot(X_pos,Y_pos,'r--')
    axis([-3 100 -50 50]);
    
    hold on;
    plot(init_x, init_y,'b','linewidth', 5)
    sum=sum-1
    plot([X_tmp(1:sum)],[Y_tmp(1:sum)], 'b-', 'linewidth',5)
    plot(X_pos,Y_pos,'r--')
    
    X_tip=X_pos(cpt);
    Y_tip=Y_pos(cpt);
    theta_tip=theta(cpt);
    X_tmp=X_pos;
    Y_tmp=Y_pos;
    
end
end