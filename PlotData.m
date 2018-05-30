function PlotData(data_dist,data_mono,Mono_Sub_switch)
global param
% Plot states (True, est. mono, est. distributed)
% ETA data
f = figure(1);
clf(f);
columns = sum(param.enabled);
rows = 6;

% AUV ETA data
if (param.enabled(1))
    titles = {{'AUV','NORTH'}, {'EAST'}, {'DOWN'}, {'ROLL '}, {'PITCH'}, {'YAW'}};
    ylabels = {'N (m)', 'E (m)', 'D (m)', '$\phi$ ($^{\circ}$)', '$\theta$ ($^{\circ}$)', '$\psi$ ($^{\circ}$)'};
    xlabels = {'','','','','','Time (s)'};
    scales = [1,1,1,180/pi,180/pi,180/pi,1,1,1,180/pi,180/pi,180/pi]; 
    for row=1:rows
        subplot(rows,columns,(row-1)*columns + 1);
        if Mono_Sub_switch == 0
            plot(data_dist.AUV.t, data_dist.AUV.X(row,:)*scales(row), data_dist.AUV.t, data_dist.AUV.Xf(row,:)*scales(row));
            grid on;
            t = titles(row);
            title(t{:}, 'Interpreter', 'latex');
            ylabel(ylabels(row), 'Interpreter', 'latex');
            xlabel(xlabels(row), 'Interpreter', 'latex');
            legend('True', 'Filtered - Distributed');
        elseif Mono_Sub_switch == 1
            plot(data_mono.AUV.t, data_mono.AUV.X(row,:)*scales(row), data_mono.AUV.t, data_mono.AUV.Xf(row,:)*scales(row));
            grid on;
            t = titles(row);
            title(t{:}, 'Interpreter', 'latex');
            ylabel(ylabels(row), 'Interpreter', 'latex');
            xlabel(xlabels(row), 'Interpreter', 'latex');
            legend('True', 'Filtered - Monolithic');
        elseif Mono_Sub_switch == 2
            plot(data_mono.AUV.t, data_mono.AUV.X(row,:)*scales(row), data_mono.AUV.t, data_mono.AUV.Xf(row,:)*scales(row),data_dist.AUV.t, data_dist.AUV.Xf(row,:)*scales(row));
            grid on;
            t = titles(row);
            title(t{:}, 'Interpreter', 'latex');
            ylabel(ylabels(row), 'Interpreter', 'latex');
            xlabel(xlabels(row), 'Interpreter', 'latex');
            legend('True', 'Filtered - Monolithic', 'Filtered - Distributed');
        end
    end
end

% WAMV ETA data
if (param.enabled(2))
    titles = {{'WAMV','NORTH'}, {'EAST'}, {'DOWN'}, {'ROLL '}, {'PITCH'}, {'YAW'}};
    ylabels = {'N (m)', 'E (m)', 'D (m)', '$\phi$ ($^{\circ}$)', '$\theta$ ($^{\circ}$)', '$\psi$ ($^{\circ}$)'};
    xlabels = {'','','','','','Time (s)'};
    scales = [1,1,1,180/pi,180/pi,180/pi,1,1,1,180/pi,180/pi,180/pi]; 
    for row=1:rows
        subplot(rows,columns,(row-1)*columns + sum(param.enabled(1:2)));
        if Mono_Sub_switch == 0
            plot(data_dist.WAMV.t, data_dist.WAMV.X(row,:)*scales(row), data_dist.WAMV.t, data_dist.WAMV.Xf(row,:)*scales(row));
            grid on;
            t = titles(row);
            title(t{:}, 'Interpreter', 'latex');
            ylabel(ylabels(row), 'Interpreter', 'latex');
            xlabel(xlabels(row), 'Interpreter', 'latex');
            legend('True', 'Filtered - Distributed');
        elseif Mono_Sub_switch == 1
            plot(data_mono.WAMV.t, data_mono.WAMV.X(row,:)*scales(row), data_mono.WAMV.t, data_mono.WAMV.Xf(row,:)*scales(row));
            grid on;
            t = titles(row);
            title(t{:}, 'Interpreter', 'latex');
            ylabel(ylabels(row), 'Interpreter', 'latex');
            xlabel(xlabels(row), 'Interpreter', 'latex');
            legend('True', 'Filtered - Monolithic');
        elseif Mono_Sub_switch == 2
            plot(data_mono.WAMV.t, data_mono.WAMV.X(row,:)*scales(row), data_mono.WAMV.t, data_mono.WAMV.Xf(row,:)*scales(row),data_dist.WAMV.t, data_dist.WAMV.Xf(row,:)*scales(row));
            grid on;
            t = titles(row);
            title(t{:}, 'Interpreter', 'latex');
            ylabel(ylabels(row), 'Interpreter', 'latex');
            xlabel(xlabels(row), 'Interpreter', 'latex');
            legend('True', 'Filtered - Monolithic', 'Filtered - Distributed');
        end
    end
end
% --- QUAD ETA data
if (param.enabled(3))
    titles = {{'QUAD','NORTH'}, {'EAST'}, {'DOWN'}, {'ROLL '}, {'PITCH'}, {'YAW'}};
    ylabels = {'N (m)', 'E (m)', 'D (m)', '$\phi$ ($^{\circ}$)', '$\theta$ ($^{\circ}$)', '$\psi$ ($^{\circ}$)'};
    xlabels = {'','','','','','Time (s)'};
    scales = [1,1,1,180/pi,180/pi,180/pi,1,1,1,180/pi,180/pi,180/pi]; 
    for row=1:rows
        subplot(rows,columns,(row-1)*columns + sum(param.enabled));
        if Mono_Sub_switch == 0
            plot(data_dist.QUAD.t, data_dist.QUAD.X(row,:)*scales(row), data_dist.QUAD.t, data_dist.QUAD.Xf(row,:)*scales(row));
            grid on;
            t = titles(row);
            title(t{:}, 'Interpreter', 'latex');
            ylabel(ylabels(row), 'Interpreter', 'latex');
            xlabel(xlabels(row), 'Interpreter', 'latex');
            legend('True', 'Filtered - Distributed');
            
        elseif Mono_Sub_switch == 1
            plot(data_mono.QUAD.t, data_mono.QUAD.X(row,:)*scales(row), data_mono.QUAD.t, data_mono.QUAD.Xf(row,:)*scales(row));
            grid on;
            t = titles(row);
            title(t{:}, 'Interpreter', 'latex');
            ylabel(ylabels(row), 'Interpreter', 'latex');
            xlabel(xlabels(row), 'Interpreter', 'latex');
            legend('True', 'Filtered - Monolithic');
            
        elseif Mono_Sub_switch == 2
            plot(data_mono.QUAD.t, data_mono.QUAD.X(row,:)*scales(row), data_mono.QUAD.t, data_mono.QUAD.Xf(row,:)*scales(row),data_dist.QUAD.t, data_dist.QUAD.Xf(row,:)*scales(row));
            grid on;
            t = titles(row);
            title(t{:}, 'Interpreter', 'latex');
            ylabel(ylabels(row), 'Interpreter', 'latex');
            xlabel(xlabels(row), 'Interpreter', 'latex');
            legend('True', 'Filtered - Monolithic', 'Filtered - Distributed');
        end
    end
end
% ---------- NU data
f = figure(2);
clf(f);
% ---------- AUV NU data
if (param.enabled(1))
    titles = {{'AUV';'SURGE'}, 'SWAY', 'HEAVE', 'ROLL RATE', 'PITCH RATE', 'YAW RATE'};
    ylabels = {'U (m)', 'V (m)', 'W (m)', 'P ($^{\circ}$/s)', 'Q ($^{\circ}$/s)', 'R ($^{\circ}$/s)'};
    xlabels = {'','','','','','Time (s)'};
    scales = [1,1,1,180/pi,180/pi,180/pi,1,1,1,180/pi,180/pi,180/pi]; 
    for row=1:rows
        subplot(rows,columns,(row-1)*columns + 1);
<<<<<<< HEAD
<<<<<<< HEAD
        hold on;
        plot(data.AUV.t, data.AUV.X(row + 6,:)*scales(row), data.AUV.t, data.AUV.Xf(row+6,:)*scales(row));
        if (row > 3)
            plot(data.AUV.t, data.AUV.gyrobias(row-3, :)*scales(row));
        end
        grid on;
        t = titles(row);
        title(t{:}, 'Interpreter', 'latex');
        ylabel(ylabels(row), 'Interpreter', 'latex');
        xlabel(xlabels(row), 'Interpreter', 'latex');
        legend('True', 'Filtered - Monolithic', 'Filtered - Distributed');
        hold off;
=======
=======
>>>>>>> 9cb5aa03f0120a7a4d5036432d6cb270fcdfd97a
        if Mono_Sub_switch == 0
            plot(data_dist.AUV.t, data_dist.AUV.X(row + 6,:)*scales(row), data_dist.AUV.t, data_dist.AUV.Xf(row+6,:)*scales(row));
            grid on;
            t = titles(row);
            title(t{:}, 'Interpreter', 'latex');
            ylabel(ylabels(row), 'Interpreter', 'latex');
            xlabel(xlabels(row), 'Interpreter', 'latex');
            legend('True', 'Filtered - Distributed');
            
        elseif Mono_Sub_switch == 1
            plot(data_mono.AUV.t, data_mono.AUV.X(row + 6,:)*scales(row), data_mono.AUV.t, data_mono.AUV.Xf(row+6,:)*scales(row));
            grid on;
            t = titles(row);
            title(t{:}, 'Interpreter', 'latex');
            ylabel(ylabels(row), 'Interpreter', 'latex');
            xlabel(xlabels(row), 'Interpreter', 'latex');
            legend('True', 'Filtered - Monolithic');   
            
        elseif Mono_Sub_switch == 2
            plot(data_mono.AUV.t, data_mono.AUV.X(row + 6,:)*scales(row), data_mono.AUV.t, data_mono.AUV.Xf(row+6,:)*scales(row), data_dist.AUV.t, data_dist.AUV.Xf(row+6,:)*scales(row));
            grid on;
            t = titles(row);
            title(t{:}, 'Interpreter', 'latex');
            ylabel(ylabels(row), 'Interpreter', 'latex');
            xlabel(xlabels(row), 'Interpreter', 'latex');
            legend('True', 'Filtered - Monolithic', 'Filtered - Distributed');   
        end
<<<<<<< HEAD
>>>>>>> 9cb5aa03f0120a7a4d5036432d6cb270fcdfd97a
=======
>>>>>>> 9cb5aa03f0120a7a4d5036432d6cb270fcdfd97a
    end

end
% ---------- WAMV NU data
if (param.enabled(2))
    titles = {{'WAMV';'SURGE'}, 'SWAY', 'HEAVE', 'ROLL RATE', 'PITCH RATE', 'YAW RATE'};
    ylabels = {'U (m)', 'V (m)', 'W (m)', 'P ($^{\circ}$/s)', 'Q ($^{\circ}$/s)', 'R ($^{\circ}$/s)'};
    xlabels = {'','','','','','Time (s)'};
    scales = [1,1,1,180/pi,180/pi,180/pi,1,1,1,180/pi,180/pi,180/pi]; 
    for row=1:rows
        subplot(rows,columns,(row-1)*columns + sum(param.enabled(1:2)));
<<<<<<< HEAD
<<<<<<< HEAD
        hold on;
        plot(data.WAMV.t, data.WAMV.X(row + 6,:)*scales(row), data.WAMV.t, data.WAMV.Xf(row+6,:)*scales(row));
        if (row > 3)
            plot(data.WAMV.t, data.WAMV.gyrobias(row-3, :)*scales(row));
        end
        grid on;
        t = titles(row);
        title(t{:}, 'Interpreter', 'latex');
        ylabel(ylabels(row), 'Interpreter', 'latex');
        xlabel(xlabels(row), 'Interpreter', 'latex');
        legend('True', 'Filtered - Monolithic', 'Filtered - Distributed');
        hold off;
=======
=======
>>>>>>> 9cb5aa03f0120a7a4d5036432d6cb270fcdfd97a
        if Mono_Sub_switch == 0
            plot(data_dist.WAMV.t, data_dist.WAMV.X(row + 6,:)*scales(row), data_dist.WAMV.t, data_dist.WAMV.Xf(row+6,:)*scales(row));
            grid on;
            t = titles(row);
            title(t{:}, 'Interpreter', 'latex');
            ylabel(ylabels(row), 'Interpreter', 'latex');
            xlabel(xlabels(row), 'Interpreter', 'latex');
            legend('True', 'Filtered - Distributed');
            
        elseif Mono_Sub_switch == 1
            plot(data_mono.WAMV.t, data_mono.WAMV.X(row + 6,:)*scales(row), data_mono.WAMV.t, data_mono.WAMV.Xf(row+6,:)*scales(row));
            grid on;
            t = titles(row);
            title(t{:}, 'Interpreter', 'latex');
            ylabel(ylabels(row), 'Interpreter', 'latex');
            xlabel(xlabels(row), 'Interpreter', 'latex');
            legend('True', 'Filtered - Monolithic');   
            
        elseif Mono_Sub_switch == 2
            plot(data_mono.WAMV.t, data_mono.WAMV.X(row + 6,:)*scales(row), data_mono.WAMV.t, data_mono.WAMV.Xf(row+6,:)*scales(row), data_dist.WAMV.t, data_dist.WAMV.Xf(row+6,:)*scales(row));
            grid on;
            t = titles(row);
            title(t{:}, 'Interpreter', 'latex');
            ylabel(ylabels(row), 'Interpreter', 'latex');
            xlabel(xlabels(row), 'Interpreter', 'latex');
            legend('True', 'Filtered - Monolithic', 'Filtered - Distributed');   
        end
<<<<<<< HEAD
>>>>>>> 9cb5aa03f0120a7a4d5036432d6cb270fcdfd97a
=======
>>>>>>> 9cb5aa03f0120a7a4d5036432d6cb270fcdfd97a
    end
end
% ---------- QUAD NU data
if (param.enabled(3))
    titles = {{'QUAD';'SURGE'}, 'SWAY', 'HEAVE', 'ROLL RATE', 'PITCH RATE', 'YAW RATE'};
    ylabels = {'U (m)', 'V (m)', 'W (m)', 'P ($^{\circ}$/s)', 'Q ($^{\circ}$/s)', 'R ($^{\circ}$/s)'};
    xlabels = {'','','','','','Time (s)'};
    scales = [1,1,1,180/pi,180/pi,180/pi,1,1,1,180/pi,180/pi,180/pi]; 
    for row=1:rows
        subplot(rows,columns,(row-1)*columns + sum(param.enabled));
<<<<<<< HEAD
<<<<<<< HEAD
        hold on;
        plot(data.QUAD.t, data.QUAD.X(row + 6,:)*scales(row), data.QUAD.t, data.QUAD.Xf(row+6,:)*scales(row));
        if (row > 3)
            plot(data.QUAD.t, data.QUAD.gyrobias(row-3, :)*scales(row));
        end
        grid on;
        t = titles(row);
        title(t{:}, 'Interpreter', 'latex');
        ylabel(ylabels(row), 'Interpreter', 'latex');
        xlabel(xlabels(row), 'Interpreter', 'latex');
        legend('True', 'Filtered - Monolithic', 'Filtered - Distributed');
        hold off;
=======
=======
>>>>>>> 9cb5aa03f0120a7a4d5036432d6cb270fcdfd97a
        if Mono_Sub_switch == 0
            plot(data_dist.QUAD.t, data_dist.QUAD.X(row + 6,:)*scales(row), data_dist.QUAD.t, data_dist.QUAD.Xf(row+6,:)*scales(row));
            grid on;
            t = titles(row);
            title(t{:}, 'Interpreter', 'latex');
            ylabel(ylabels(row), 'Interpreter', 'latex');
            xlabel(xlabels(row), 'Interpreter', 'latex');
            legend('True', 'Filtered - Distributed');
            
        elseif Mono_Sub_switch == 1
            plot(data_mono.QUAD.t, data_mono.QUAD.X(row + 6,:)*scales(row), data_mono.QUAD.t, data_mono.QUAD.Xf(row+6,:)*scales(row));
            grid on;
            t = titles(row);
            title(t{:}, 'Interpreter', 'latex');
            ylabel(ylabels(row), 'Interpreter', 'latex');
            xlabel(xlabels(row), 'Interpreter', 'latex');
            legend('True', 'Filtered - Monolithic');   
            
        elseif Mono_Sub_switch == 2
            plot(data_mono.QUAD.t, data_mono.QUAD.X(row + 6,:)*scales(row), data_mono.QUAD.t, data_mono.QUAD.Xf(row+6,:)*scales(row), data_dist.QUAD.t, data_dist.QUAD.Xf(row+6,:)*scales(row));
            grid on;
            t = titles(row);
            title(t{:}, 'Interpreter', 'latex');
            ylabel(ylabels(row), 'Interpreter', 'latex');
            xlabel(xlabels(row), 'Interpreter', 'latex');
            legend('True', 'Filtered - Monolithic', 'Filtered - Distributed');   
        end
<<<<<<< HEAD
>>>>>>> 9cb5aa03f0120a7a4d5036432d6cb270fcdfd97a
=======
>>>>>>> 9cb5aa03f0120a7a4d5036432d6cb270fcdfd97a
    end
end

% % ---------- TauB data
% figure(3);
% % ---------- AUV TauB data
% if (param.enabled(1))
%     titles = {{'AUV';'SURGE Forces'}, 'SWAY Forces', 'HEAVE Forces', 'ROLL Forces', 'PITCH Forces', 'YAW Forces'};
%     ylabels = {'X (N)', 'V (N)', 'Z (N)', 'K (Nm)', 'M (Nm)', 'N (Nm)'};
%     xlabels = {'','','','','','Time (s)'};
%     
%     TauB = param.AUV.MRB*data.AUV.dnu;
%     
%     for row=1:rows
%         subplot(rows,columns,(row-1)*columns + 1);
%         
%         plot(data.AUV.t, TauB(row,:));% data.QUAD.Xf.t, data.QUAD.Xf.
%         grid on;
%         t = titles(row);
%         title(t{:}, 'Interpreter', 'latex');
%         ylabel(ylabels(row), 'Interpreter', 'latex');
%         xlabel(xlabels(row), 'Interpreter', 'latex');
%         legend('True', 'Filtered - Monolithic', 'Filtered - Distributed');
%     end
% 
% end
% % ---------- WAMV NU data
% if (param.enabled(2))
%     titles = {{'WAMV';'SURGE Forces'}, 'SWAY Forces', 'HEAVE Forces', 'ROLL Forces', 'PITCH Forces', 'YAW Forces'};
%     ylabels = {'X (N)', 'V (N)', 'Z (N)', 'K (Nm)', 'M (Nm)', 'N (Nm)'};
%     xlabels = {'','','','','','Time (s)'};
%     
%     TauB = param.WAMV.MRB*data.WAMV.dnu;
%     
%     for row=1:rows
%         subplot(rows,columns,(row-1)*columns + sum(param.enabled(1:2)));
%         
%         plot(data.WAMV.t, TauB(row,:));% data.QUAD.Xf.t, data.QUAD.Xf.
%         grid on;
%         t = titles(row);
%         title(t{:}, 'Interpreter', 'latex');
%         ylabel(ylabels(row), 'Interpreter', 'latex');
%         xlabel(xlabels(row), 'Interpreter', 'latex');
%         legend('True', 'Filtered - Monolithic', 'Filtered - Distributed');
%     end
% end
% % ---------- QUAD NU data
% if (param.enabled(3))
%     titles = {{'QUAD';'SURGE Forces'}, 'SWAY Forces', 'HEAVE Forces', 'ROLL Forces', 'PITCH Forces', 'YAW Forces'};
%     ylabels = {'X (N)', 'V (N)', 'Z (N)', 'K (Nm)', 'M (Nm)', 'N (Nm)'};
%     xlabels = {'','','','','','Time (s)'};
%     
%     TauB = param.QUAD.MRB*data.QUAD.dnu;
%     
%     for row=1:rows
%         subplot(rows,columns,(row-1)*columns + sum(param.enabled));
%         
%         plot(data.QUAD.t, TauB(row,:));% data.QUAD.Xf.t, data.QUAD.Xf.
%         grid on;
%         t = titles(row);
%         title(t{:}, 'Interpreter', 'latex');
%         ylabel(ylabels(row), 'Interpreter', 'latex');
%         xlabel(xlabels(row), 'Interpreter', 'latex');
%         legend('True', 'Filtered - Monolithic', 'Filtered - Distributed');
%     end
% end