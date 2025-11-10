function graph(R, q_total, t_total, scan_log, vref)
% Gráficos compactos: perfiles x,y,z (pos/vel/acel) y panel de validación.

    [P,V,A] = cartesian_profiles(R, q_total, t_total);

    % --- Perfiles x,y,z ---

    figure('Name','Perfiles cartesianos','Color','w');

    t = t_total(:);
    labels = {'x','y','z'};

    for d=1:3
        subplot(3,3,(d-1)*3+1); plot(t,P(:,d),'LineWidth',1.2); grid on; ylabel([labels{d} ' [m]']); if d==1, title('Posición'); end
        
        subplot(3,3,(d-1)*3+2); plot(t,V(:,d),'LineWidth',1.2); grid on; if d==1, title('Velocidad'); end
        
        subplot(3,3,(d-1)*3+3); plot(t,A(:,d),'LineWidth',1.2); grid on; if d==1, title('Aceleración'); end
    end

    xlabel(subplot(3,3,7), 't [s]'); 
    xlabel(subplot(3,3,8), 't [s]'); 
    xlabel(subplot(3,3,9), 't [s]');

    % --- Panel de validación: |v_tip|, v_proy y error lateral ---
    figure('Name','Validación de scan','Color','w');
    
    vt = vecnorm(V,2,2);

    vproj = nan(size(t));
    elat  = nan(size(t));
    
    for i=1:numel(scan_log)
        k0=scan_log(i).k0; k1=scan_log(i).k1;
        
        p0 = transl(scan_log(i).T0).';
        p1 = transl(scan_log(i).T1).';
        
        d  = (p1-p0)/norm(p1-p0);
        
        idx = k0:k1;
        
        vproj(idx) = V(idx,:)*d;
        E = P(idx,:) - p0.';
        proj = (E*d)*d.';                   % (Nx1)*(1x3) -> Nx3
        elat(idx) = vecnorm(E - proj, 2, 2);
    end

    subplot(3,1,1); plot(t,vt,'LineWidth',1.2); grid on; ylabel('|v_{tip}| [m/s]');
    
    subplot(3,1,2); plot(t,vproj,'LineWidth',1.2); grid on; ylabel('v_{||} [m/s]'); hold on;
    
    if nargin>=5 && ~isempty(vref), yline(vref,'k--'); end
    
    subplot(3,1,3); plot(t,1e3*elat,'LineWidth',1.2); grid on; ylabel('e_{\perp} [mm]'); xlabel('t [s]');
end

% ==================== SUBFUNCIONES ====================

function [P,V,A] = cartesian_profiles(R,q,t)
    N=size(q,1);
    P=zeros(N,3);
    for k=1:N, T=R.fkine(q(k,:)); P(k,:)=transl(T); end

    dt = mean(diff(t));
    V = deriv_smooth(t, P, 0.30, 3);       % Savitzky-Golay + central difference
    A = deriv_smooth(t, V, 0.30, 3);
    end

    function dy = deriv_smooth(t, y, win_sec, poly)
    if nargin<3, win_sec=0.30; end
    if nargin<4, poly=3; end
    N=size(y,1); D=size(y,2); dy=zeros(N,D);
    t=t(:); dtm=mean(diff(t)); frame=max(3,2*floor((win_sec/dtm)/2)+1); frame=min(frame,2*floor((N-1)/2)+1);
    if exist('sgolayfilt','file')==2, ys=sgolayfilt(y,poly,frame); else, ys=y; end
    dy(1,:)=(ys(2,:)-ys(1,:))/max(t(2)-t(1),eps);
    dy(end,:)=(ys(end,:)-ys(end-1,:))/max(t(end)-t(end-1),eps);
    for k=2:N-1, dy(k,:)=(ys(k+1,:)-ys(k-1,:))/max(t(k+1)-t(k-1),eps); end
end
