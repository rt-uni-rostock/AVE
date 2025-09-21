function [F, B] = ASVIdent(trials, initialF, initialB, maskIdentF, maskIdentB, weightUVR, plotComparison, simulateFreeResponse)
    %ave.ASVIdent Run an optimization to find parameters of a non-linear grey-box model of an autonomous surface vehicle (ASV) based
    % on experimental data. The model structure is as follows:
    % 
    % ( u_dot )
    % ( v_dot ) = F * [u; v; r; v*r; u*r; u*v; u^2; v^2; r^2; u^3; v^3; r^3] + B * [X; Y; N]
    % ( r_dot )
    % 
    % Here, [u; v; r] denotes the body-fixed velocity vector and [X; Y; N] denotes the input force. F and B are 3-by-12 and
    % 3-by-3 coefficient matrices, whose parameters are to be found.
    % 
    % 
    % PARAMETERS
    % trials               ... Cell-array of structs, where each struct in a cell must contain the following fields:
    %                          sampletime ... Scalar positive value representing the fixed-step sample time of the data.
    %                          X          ... n-by-1 vector of input data X for the trial.
    %                          Y          ... n-by-1 vector of input data Y for the trial.
    %                          N          ... n-by-1 vector of input data N for the trial.
    %                          u          ... n-by-1 vector of output data u for the trial.
    %                          v          ... n-by-1 vector of output data v for the trial.
    %                          r          ... n-by-1 vector of output data r for the trial.
    % initialF             ... 3-by-12 matrix representing the initial guess for coefficient matrix F.
    % initialB             ... 3-by-3 matrix representing the initial guess for coefficient matrix B.
    % maskIdentF           ... 3-by-12 matrix representing the entries of the F-matrix to be identified. A zero-value indicates that the
    %                          corresponding coefficient stays fixed during the optimization.
    % maskIdentB           ... 3-by-3 matrix representing the entries of the B-matrix to be identified. A zero-value indicates that the
    %                          corresponding coefficient stays fixed during the optimization.
    % weightUVR            ... (Optional) 3-by-1 vector representing the weighting for u, v and r. The default value is [1; 1; 1;].
    % plotComparison       ... (Optional) Boolean value that indicates whether to create a final comparison plot or not. The default value is TRUE.
    % simulateFreeResponse ... (Optional) Check the free-response function by doing a numerical simulation for different initial velocities and plot the resulting velocities. This parameter indicates the simulation time in seconds. The default value is -1.
    % 
    % 
    % GENERAL NOTES
    % - consider downsampling the trial data to speed up computation, especially for large datasets
    % - try to find a good initial guess by only assigning non-zero values to the diagonal of F(:,1:3) and B (a bad initial guess may lead to large computation times with bad results)
    % - start estimating those diagonals, then extend model to include diag(F(:,10:12)) and full B
    % - keep in mind that some parameter combinations might produce a model whose equilibrium is no longer at u=v=r=0, especially when using off-diagonal elements
    % 
    % 
    % EXAMPLE CODE
    % F = [-0.03 0 0 0 0 0 0 0 0 0 0 0; 0 -0.04 0 0 0 0 0 0 0 0 0 0; 0 0 -0.02 0 0 0 0 0 0 0 0 0];
    % B = [0.0003 0 0; 0 0.0003 0; 0 0 0.00007];
    % maskIdentF = [
    %     1 0 0 0 0 0 0 0 0 0 0 0;
    %     0 1 0 0 0 0 0 0 0 0 0 0;
    %     0 0 1 0 0 0 0 0 0 0 0 0
    % ];
    % maskIdentB = [
    %     1 0 0;
    %     0 1 0;
    %     0 0 1
    % ];
    % [F,B] = ASVIdent(trials, F, B, maskIdentF, maskIdentB);

    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % Argument validation
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    arguments (Input)
        trials cell
        initialF (3,12) double {mustBeFinite} = [-1 0 0 0 0 0 0 0 0 0 0 0; 0 -1 0 0 0 0 0 0 0 0 0 0; 0 0 -1 0 0 0 0 0 0 0 0 0]
        initialB (3,3) double {mustBeFinite} = [1 0 0; 0 1 0; 0 0 1]
        maskIdentF (3,12) = zeros(3,12)
        maskIdentB (3,3) = zeros(3)
        weightUVR (3,1) double {mustBeFinite, mustBePositive} = [1; 1; 1]
        plotComparison (1,1) logical = true
        simulateFreeResponse (1,1) double {mustBeFinite} = -1
    end
    arguments (Output)
        F (3,12) double {mustBeFinite}
        B (3,3) double {mustBeFinite}
    end
    numDatasets = numel(trials);
    for i = 1:numDatasets
        assert(isfield(trials{i},'sampletime'), ['Trial ' num2str(i) ' does not contain a field "sampletime"!']);
        assert(isfield(trials{i},'X'), ['Trial ' num2str(i) ' does not contain a field "X"!']);
        assert(isfield(trials{i},'Y'), ['Trial ' num2str(i) ' does not contain a field "Y"!']);
        assert(isfield(trials{i},'N'), ['Trial ' num2str(i) ' does not contain a field "N"!']);
        assert(isfield(trials{i},'u'), ['Trial ' num2str(i) ' does not contain a field "u"!']);
        assert(isfield(trials{i},'v'), ['Trial ' num2str(i) ' does not contain a field "v"!']);
        assert(isfield(trials{i},'r'), ['Trial ' num2str(i) ' does not contain a field "r"!']);
        assert(isscalar(trials{i}.sampletime) && isfinite(trials{i}.sampletime) && (trials{i}.sampletime > 0), ['Trial ' num2str(i) ' has an invalid sampletime!']);
        assert(~any(~isfinite(trials{i}.X)), ['Trial ' num2str(i) ' contains non-finite values for X!']);
        assert(~any(~isfinite(trials{i}.Y)), ['Trial ' num2str(i) ' contains non-finite values for Y!']);
        assert(~any(~isfinite(trials{i}.N)), ['Trial ' num2str(i) ' contains non-finite values for N!']);
        assert(~any(~isfinite(trials{i}.u)), ['Trial ' num2str(i) ' contains non-finite values for u!']);
        assert(~any(~isfinite(trials{i}.v)), ['Trial ' num2str(i) ' contains non-finite values for v!']);
        assert(~any(~isfinite(trials{i}.r)), ['Trial ' num2str(i) ' contains non-finite values for r!']);
        assert((numel(trials{i}.X) == numel(trials{i}.Y)) && (numel(trials{i}.X) == numel(trials{i}.N)) && (numel(trials{i}.X) == numel(trials{i}.u)) && (numel(trials{i}.X) == numel(trials{i}.v)) && (numel(trials{i}.X) == numel(trials{i}.r)), ['Data size of trial ' num2str(i) ' does not match!']);
    end
    assert(initialF(1,1) <= 0.0, 'Initial value for F(1,1) must be negative!');
    assert(initialF(2,2) <= 0.0, 'Initial value for F(2,2) must be negative!');
    assert(initialF(3,3) <= 0.0, 'Initial value for F(3,3) must be negative!');
    assert(initialF(1,4) <= 0.0, 'Initial value for F(1,4) must be negative!');
    assert(initialF(2,5) >= 0.0, 'Initial value for F(2,5) must be positive!');
    assert(initialF(1,10) <= 0.0, 'Initial value for F(1,10) must be negative!');
    assert(initialF(2,11) <= 0.0, 'Initial value for F(2,11) must be negative!');
    assert(initialF(3,12) <= 0.0, 'Initial value for F(3,12) must be negative!');
    assert(initialB(1,1) >= 0.0, 'Initial value for B(1,1) must be positive!');
    assert(initialB(2,2) >= 0.0, 'Initial value for B(2,2) must be positive!');
    assert(initialB(3,3) >= 0.0, 'Initial value for B(3,3) must be positive!');


    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % Prepare datasets
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    fprintf('[ASV IDENT] Preparing %d datasets ...\n', numDatasets);
    tic();
    initialValues = zeros(3,numDatasets);
    weightU = weightUVR(1);
    weightV = weightUVR(2);
    weightR = weightUVR(3);
    fprintf('[ASV IDENT] Applying data weights: weightU=%f, weightV=%f, weightR=%f\n', weightU, weightV, weightR);
    for i = 1:numDatasets
        if(1 == i)
            datasets = iddata([trials{i}.u*weightU trials{i}.v*weightV trials{i}.r*weightR], [trials{i}.X trials{i}.Y trials{i}.N], trials{i}.sampletime);
        else
            datasets = merge(datasets, iddata([trials{i}.u*weightU trials{i}.v*weightV trials{i}.r*weightR], [trials{i}.X trials{i}.Y trials{i}.N], trials{i}.sampletime));
        end
        initialValues(:,i) = [trials{i}.u(1); trials{i}.v(1); trials{i}.r(1)];
    end


    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % Prepare solver
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    fprintf('[ASV IDENT] Preparing solver ...\n');
    fixed_matF = ~boolean(maskIdentF);
    fixed_matB = ~boolean(maskIdentB);
    parameterValue = { weightU , weightV , weightR ,   initialF(1,1),   initialF(1,2),   initialF(1,3),   initialF(1,4),   initialF(1,5),   initialF(1,6),   initialF(1,7),   initialF(1,8),   initialF(1,9),   initialF(1,10),   initialF(1,11),   initialF(1,12),   initialF(2,1),   initialF(2,2),   initialF(2,3),   initialF(2,4),   initialF(2,5),   initialF(2,6),   initialF(2,7),   initialF(2,8),   initialF(2,9),   initialF(2,10),   initialF(2,11),   initialF(2,12),   initialF(3,1),   initialF(3,2),   initialF(3,3),   initialF(3,4),   initialF(3,5),   initialF(3,6),   initialF(3,7),   initialF(3,8),   initialF(3,9),   initialF(3,10),   initialF(3,11),   initialF(3,12),   initialB(1,1),   initialB(1,2),   initialB(1,3),   initialB(2,1),   initialB(2,2),   initialB(2,3),   initialB(3,1),   initialB(3,2),   initialB(3,3)};
    parameterFixed = {   true  ,   true  ,   true  , fixed_matF(1,1), fixed_matF(1,2), fixed_matF(1,3), fixed_matF(1,4), fixed_matF(1,5), fixed_matF(1,6), fixed_matF(1,7), fixed_matF(1,8), fixed_matF(1,9), fixed_matF(1,10), fixed_matF(1,11), fixed_matF(1,12), fixed_matF(2,1), fixed_matF(2,2), fixed_matF(2,3), fixed_matF(2,4), fixed_matF(2,5), fixed_matF(2,6), fixed_matF(2,7), fixed_matF(2,8), fixed_matF(2,9), fixed_matF(2,10), fixed_matF(2,11), fixed_matF(2,12), fixed_matF(3,1), fixed_matF(3,2), fixed_matF(3,3), fixed_matF(3,4), fixed_matF(3,5), fixed_matF(3,6), fixed_matF(3,7), fixed_matF(3,8), fixed_matF(3,9), fixed_matF(3,10), fixed_matF(3,11), fixed_matF(3,12), fixed_matB(1,1), fixed_matB(1,2), fixed_matB(1,3), fixed_matB(2,1), fixed_matB(2,2), fixed_matB(2,3), fixed_matB(3,1), fixed_matB(3,2), fixed_matB(3,3)};
    parameterName  = {'weightU','weightV','weightR',           'f11',           'f12',           'f13',           'f14',           'f15',           'f16',           'f17',           'f18',           'f19',            'f1a',            'f1b',            'f1c',           'f21',           'f22',           'f23',           'f24',           'f25',           'f26',           'f27',           'f28',           'f29',            'f2a',            'f2b',            'f2c',           'f31',           'f32',           'f33',           'f34',           'f35',           'f36',           'f37',           'f38',           'f39',            'f3a',            'f3b',            'f3c',           'b11',           'b12',           'b13',           'b21',           'b22',           'b23',           'b31',           'b32',           'b33'};
    parameterMin   = { weightU , weightV , weightR ,            -inf,            -inf,            -inf,               0,            -inf,            -inf,            -inf,            -inf,            -inf,             -inf,             -inf,             -inf,            -inf,            -inf,            -inf,            -inf,            -inf,            -inf,            -inf,            -inf,            -inf,             -inf,             -inf,             -inf,            -inf,            -inf,            -inf,            -inf,            -inf,            -inf,            -inf,            -inf,            -inf,             -inf,             -inf,             -inf,               0,            -inf,            -inf,            -inf,               0,            -inf,            -inf,            -inf,               0};
    parameterMax   = { weightU , weightV , weightR ,               0,             inf,             inf,             inf,             inf,             inf,             inf,             inf,             inf,                0,              inf,              inf,             inf,               0,             inf,             inf,               0,             inf,             inf,             inf,             inf,              inf,                0,              inf,             inf,             inf,               0,             inf,             inf,             inf,             inf,             inf,             inf,              inf,              inf,                0,             inf,             inf,             inf,             inf,             inf,             inf,             inf,             inf,             inf};
    
    % Create model
    mdl = idnlgrey(@ave.detail.ASVIdentModel, [3 3 3], parameterValue, {initialValues(1,:),initialValues(2,:),initialValues(3,:)}, 0);
    mdl = setpar(mdl, 'Name', parameterName);
    mdl = setpar(mdl, 'Minimum', parameterMin);
    mdl = setpar(mdl, 'Maximum', parameterMax);
    mdl = setpar(mdl, 'Fixed', parameterFixed);
    mdl.InitialStates(1).Fixed = true(1,numDatasets);
    mdl.InitialStates(2).Fixed = true(1,numDatasets);
    mdl.InitialStates(3).Fixed = true(1,numDatasets);
    
    % Solver options
    opt = nlgreyestOptions;
    opt.Display = 'on';
    opt.SearchOptions.MaxIterations = 100;
    opt.SearchMethod = 'lsqnonlin';
    opt.SearchOptions.Advanced.UseParallel = true;


    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % Run optimization
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    fprintf('[ASV IDENT] Run optimization (maxIterations=%d) ...\n', opt.SearchOptions.MaxIterations);
    mdl = nlgreyest(datasets, mdl, opt);
    fprintf('[ASV IDENT] Optimization finished\n');

    
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % Get result
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    result = get(mdl);
    F = initialF;
    B = initialB;
    for i = 1:3
        for j = 1:12
            F(i,j) = result.Parameters(3 + j + 12 * (i - 1)).Value;
        end
        for k = 1:3
            B(i,k) = result.Parameters(3 + 36 + k + 3*(i-1)).Value;
        end
    end

    % print result
    fprintf('F = [\n');
    for i = 1:3
        fprintf('   ');
        for j = 1:12
            fprintf(' %g',F(i,j));
        end
        fprintf('\n');
    end
    fprintf('];\nB = [\n');
    for i = 1:3
        fprintf('   ');
        for j = 1:3
            fprintf(' %g',B(i,j));
        end
        fprintf('\n');
    end
    fprintf('];\n');


    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % Comparison plot
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    if(plotComparison)
        fprintf('[ASV IDENT] Creating comparison plot\n');
        figure(20250908); clf;
        i = floor(sqrt(double(numDatasets)));
        j = ceil(double(numDatasets) / i);
        for n = 1:double(numDatasets)
            subplot(i,j,n); compare(datasets(:,:,:,n), mdl);
        end
        delete(findobj('type','legend'));
    end


    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % Check free-response function
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    if(simulateFreeResponse > 0)
        fprintf('[ASV IDENT] Simulating free-response\n');
        umax = 0;
        vmax = 0;
        rmax = 0;
        for i = 1:numDatasets
            umax = max(umax, max(abs(trials{i}.u)));
            vmax = max(vmax, max(abs(trials{i}.v)));
            rmax = max(rmax, max(abs(trials{i}.r)));
        end
        breakpointsU = linspace(-umax, +umax, 20);
        breakpointsV = linspace(-vmax, +vmax, 20);
        breakpointsR = linspace(-rmax, +rmax, 20);

        freeResponseFunction = @(t, y0)ave.detail.ASVIdentModel(t,y0,zeros(8,1),1,1,1,F(1,1),F(1,2),F(1,3),F(1,4),F(1,5),F(1,6),F(1,7),F(1,8),F(1,9),F(1,10),F(1,11),F(1,12),F(2,1),F(2,2),F(2,3),F(2,4),F(2,5),F(2,6),F(2,7),F(2,8),F(2,9),F(2,10),F(2,11),F(2,12),F(3,1),F(3,2),F(3,3),F(3,4),F(3,5),F(3,6),F(3,7),F(3,8),F(3,9),F(3,10),F(3,11),F(3,12),B(1,1),B(1,2),B(1,3),B(2,1),B(2,2),B(2,3),B(3,1),B(3,2),B(3,3));
        uvr = zeros(numel(breakpointsR)*numel(breakpointsV)*numel(breakpointsU), 3);
        k = 0;
        for ir = 1:numel(breakpointsR)
            for iv = 1:numel(breakpointsV)
                for iu = 1:numel(breakpointsU)
                    u0 = breakpointsU(iu);
                    v0 = breakpointsV(iv);
                    r0 = breakpointsR(ir);
                    [~,y] = ode45(freeResponseFunction,[0 simulateFreeResponse],[u0; v0; r0]);
                    k = k + 1;
                    uvr(k,:) = y(end,:);
                end
            end
        end

        figure(20250909); clf;
        grid on;
        xlabel('u (m/s)');
        ylabel('v (m/s)');
        zlabel('r (rad/s)');
        xlim([min(min(breakpointsU), min(uvr(:,1))), max(max(breakpointsU), max(uvr(:,1)))]);
        ylim([min(min(breakpointsV), min(uvr(:,2))), max(max(breakpointsV), max(uvr(:,2)))]);
        zlim([min(min(breakpointsR), min(uvr(:,3))), max(max(breakpointsR), max(uvr(:,3)))]);
        hold on;
        line(xlim(),[0 0],[0 0], 'Color', '#ccc', 'LineWidth',1);
        line([0 0],ylim(),[0 0], 'Color', '#ccc', 'LineWidth',1);
        line([0 0],[0 0],zlim(), 'Color', '#ccc', 'LineWidth',1);
        scatter3(uvr(:,1), uvr(:,2), uvr(:,3));
        grid on;
        view(-30,30);
    end
    fprintf('[ASV IDENT] Done (%f seconds)\n',toc());
end
