function PlotData(data)
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
        plot(data.t, data.AUV.X(row,:)*scales(row), data.t, data.AUV.Xf(row,:)*scales(row));
        grid on;
        t = titles(row);
        title(t{:}, 'Interpreter', 'latex');
        ylabel(ylabels(row), 'Interpreter', 'latex');
        xlabel(xlabels(row), 'Interpreter', 'latex');
        legend('True', 'Filtered - Distributed');
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
            plot(data.t, data.WAMV.X(row,:)*scales(row), data.t, data.WAMV.Xf(row,:)*scales(row));
            grid on;
            t = titles(row);
            title(t{:}, 'Interpreter', 'latex');
            ylabel(ylabels(row), 'Interpreter', 'latex');
            xlabel(xlabels(row), 'Interpreter', 'latex');
            legend('True', 'Filtered - Distributed');
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
        plot(data.t, data.QUAD.X(row,:)*scales(row), data.t, data.QUAD.Xf(row,:)*scales(row));
        grid on;
        t = titles(row);
        title(t{:}, 'Interpreter', 'latex');
        ylabel(ylabels(row), 'Interpreter', 'latex');
        xlabel(xlabels(row), 'Interpreter', 'latex');
        legend('True', 'Filtered - Distributed');
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
        plot(data.t, data.AUV.X(row+6,:)*scales(row), data.t, data.AUV.Xf(row+6,:)*scales(row));
        grid on;
        t = titles(row);
        title(t{:}, 'Interpreter', 'latex');
        ylabel(ylabels(row), 'Interpreter', 'latex');
        xlabel(xlabels(row), 'Interpreter', 'latex');
        legend('True', 'Filtered - Distributed');
    end
end

% WAMV NU data
if (param.enabled(2))
    titles = {{'WAMV';'SURGE'}, 'SWAY', 'HEAVE', 'ROLL RATE', 'PITCH RATE', 'YAW RATE'};
    ylabels = {'U (m)', 'V (m)', 'W (m)', 'P ($^{\circ}$/s)', 'Q ($^{\circ}$/s)', 'R ($^{\circ}$/s)'};
    xlabels = {'','','','','','Time (s)'};
    scales = [1,1,1,180/pi,180/pi,180/pi,1,1,1,180/pi,180/pi,180/pi]; 
    for row=1:rows
        subplot(rows,columns,(row-1)*columns + sum(param.enabled(1:2)));
        plot(data.t, data.WAMV.X(row+6,:)*scales(row), data.t, data.WAMV.Xf(row+6,:)*scales(row));
        grid on;
        t = titles(row);
        title(t{:}, 'Interpreter', 'latex');
        ylabel(ylabels(row), 'Interpreter', 'latex');
        xlabel(xlabels(row), 'Interpreter', 'latex');
        legend('True', 'Filtered - Distributed');
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
        plot(data.t, data.QUAD.X(row+6,:)*scales(row), data.t, data.QUAD.Xf(row+6,:)*scales(row));
        grid on;
        t = titles(row);
        title(t{:}, 'Interpreter', 'latex');
        ylabel(ylabels(row), 'Interpreter', 'latex');
        xlabel(xlabels(row), 'Interpreter', 'latex');
        legend('True', 'Filtered - Distributed');
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
%         plot(data.t, TauB(row,:));% data.QUAD.Xf.t, data.QUAD.Xf.
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
%         plot(data.t, TauB(row,:));% data.QUAD.Xf.t, data.QUAD.Xf.
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
%         plot(data.t, TauB(row,:));% data.QUAD.Xf.t, data.QUAD.Xf.
%         grid on;
%         t = titles(row);
%         title(t{:}, 'Interpreter', 'latex');
%         ylabel(ylabels(row), 'Interpreter', 'latex');
%         xlabel(xlabels(row), 'Interpreter', 'latex');
%         legend('True', 'Filtered - Monolithic', 'Filtered - Distributed');
%     end
% end