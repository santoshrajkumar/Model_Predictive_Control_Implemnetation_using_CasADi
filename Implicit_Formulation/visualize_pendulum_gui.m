clc; close all; clear;
%% load data 
load('X_pend.mat');
theta=X_act(:,1);
l=1;

numberOfFrames=length(X_act(:,1));
allTheFrames = cell(numberOfFrames,1);
vidHeight = 720;
vidWidth = 1280;
allTheFrames(:) = {zeros(vidHeight, vidWidth, 3, 'uint8')};
% Next get a cell array with all the colormaps.
allTheColorMaps = cell(numberOfFrames,1);
allTheColorMaps(:) = {zeros(256, 3)};
% Now combine these to make the array of structures.
myMovie = struct('cdata', allTheFrames, 'colormap', allTheColorMaps);
% Create a VideoWriter object to write the video out to a new, different file.
% writerObj = VideoWriter('problem_3.avi');
% open(writerObj);
% Need to change from the default renderer to zbuffer to get it to work right.
% openGL doesn't work and Painters is way too slow.
set(gcf, 'renderer', 'zbuffer');


for i=1:length(theta)
    plot([0 0],[0 5],'k--')
    r = 0.1;
    c = [l*sin(-theta(i)) l*cos(-theta(i))];
    g(i) = rectangle('Position',[c-r 2*r 2*r],'Curvature',[1 1], 'FaceColor', 'b', 'Edgecolor','none');
    h(i)=plot([0 l*sin(-theta(i))],[0 l*cos(-theta(i))],'b','Linewidth',10);
    rectangle('Position',[[0 0]-0.02 2*0.02 2*0.02],'Curvature',[1 1], 'FaceColor', 'g', 'Edgecolor','r');
    xlim([-1.2 1.2]);
    ylim([0 1.2]);
    axis off
    drawnow
    hold on
    % pause(0.1)

    % get frame
    thisFrame = getframe(gca);
	% Write this frame out to a new video file.
    %  	writeVideo(writerObj, thisFrame);
    if i <= numberOfFrames 
	    myMovie(i) = thisFrame;
    end
    if i<length(theta)
        delete(h(i))
        delete(g(i))
    end
end


%% writing as a video
writerObj = VideoWriter('mymov.mp4', 'MPEG-4');
writerObj.Quality = 95;
open(writerObj);
% Write out all the frames.
numberOfFrames = length(myMovie);
for frameNumber = 1 : numberOfFrames 
   writeVideo(writerObj, myMovie(frameNumber));
end
close(writerObj);





