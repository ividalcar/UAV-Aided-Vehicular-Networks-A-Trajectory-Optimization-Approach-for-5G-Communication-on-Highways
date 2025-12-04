clc
clear variables
close all

% Highway scenario
% SUMO parameters

num_iterations = 1;
snr_all_iterations = cell(num_iterations, 3);
bler_all_iterations = cell(num_iterations, 3);
throughput_all_iterations = cell(num_iterations, 3);


% "divisor" es una variable por la cual se dividen las velocidades de los UAVs, relacionada
% con <step-length value="1"/> en el archivo .sumocfg (regula las
% velocidades de los vehículos), tal que si, por ejemplo, busco que el paso
% del tiempo de los vehículos sea para 0.2 segundos, divisor debe ser 5,
% ya que 1/5 = 0.2 s

divisor=1;

for iter = 1:num_iterations
    % Iniciar SUMO con la configuración especificada
    traci.start('sumo-gui -c ./highway_bremen_tracking.sumocfg --start');

    % Definir el número de vehículos y preasignar sus IDs
    num_autos = 1;
    vehicleIDs = cell(1, num_autos);

    % Añadir vehículos a la simulación en rutas aleatorias
     for i = 1:num_autos
         vehicleIDs{i} = 'veh_1';
         depart = '0';
     end

    % UAV: fixed, circular and mobile
    % Crear el Dron Fijo como POI en la simulación
    pos_inicialX = 1010.7;  
    pos_inicialY = 767;      

    poiID_Fijo = 'Dron_Fijo';
    traci.poi.add(poiID_Fijo, pos_inicialY, pos_inicialX, [0, 255, 0, 255], 'Circle', 30);    %Recordar que por alguna razón invierten los ejes cuando se añade POI

    % Crear el Dron Circular como POI en la simulación
    poiID_Predefinido = 'Dron_Predefinido';
    traci.poi.add(poiID_Predefinido, pos_inicialY, pos_inicialX, [255, 0, 0, 255], 'Circle', 30);
    posiciones = [
        pos_inicialX, pos_inicialY;          % Punto inicial
        1062.99, 678.15;    % Primer destino
        810.23, 518.35;     % Segundo destino (primera vuelta)
        1363.55, 867.54;    % Tercer destino (segunda vuelta)
        ];

    velocidad = 20/divisor; % Velocidad constante en m/s
    posicion_actual = 1; % Índice de la posición actual
    direccion = 1; % Dirección del movimiento (1 hacia adelante, -1 hacia atrás)
    % Coordenadas iniciales
    posX_dron_predefinido = posiciones(posicion_actual, 1);
    posY_dron_predefinido = posiciones(posicion_actual, 2);

    % Crear el Dron Móvil como POI en la simulación
    poiID_Movil = 'Dron_Movil';
    traci.poi.add(poiID_Movil, pos_inicialY, pos_inicialX, [0, 0, 255, 255], 'Circle', 10);

    %Parámetros del dron móvil
    Vel_dron = 0;
    acel_max = 2.5/divisor;
    Vel_max = 23/divisor;
    if acel_max == 0
        Vel_dron = 22;
    end

    % Solo para el caso que se requiera Tracking (1 vehículo en la simulación)

    mov = 11.5/divisor; % Movimiento constante del vehículo en x, en y es *1.5; 12 = 21.6
    % Determinar si el dron está siguiendo al auto
    siguiendo_auto = false;
    tiempo = 1;

    % Initializing the metrics variables
    % Inicializar matrices y celdas para almacenar datos de la simulación
    pos3D_history_Predefinido = [];
    pos3D_history_Movil = [];
    pos3D_history_vehiculos = cell(1, num_autos);
    SNRs_Predefinido = [];
    SNRs_Fijo = [];
    SNRs_Movil = [];
    bler_movil = cell(1,num_autos);
    bler_fijo = cell(1,num_autos);
    bler_predefinido = cell(1,num_autos);
    simThroughput_vehiculos_Predefinido = cell(1, num_autos);
    maxThroughput_vehiculos_Predefinido = cell(1,num_autos);
    simThroughput_vehiculos_Fijo = cell(1, num_autos);
    maxThroughput_vehiculos_Fijo = cell(1,num_autos);
    simThroughput_vehiculos_Movil = cell(1, num_autos);
    maxThroughput_vehiculos_Movil = cell(1,num_autos);
    SNR_vehiculos_Predefinido = cell(num_autos, 1);
    throughput_vehiculos_Predefinido = cell(num_autos, 1);
    SNR_vehiculos_Fijo = cell(num_autos, 1);
    throughput_vehiculos_Fijo = cell(num_autos, 1);
    SNR_vehiculos_Movil = cell(num_autos, 1);
    throughput_vehiculos_Movil = cell(num_autos, 1);
    promedioSNR_Predefinido = [];
    promedioSNR_Fijo = [];
    promedioSNR_Movil = [];

    for i = 1:num_autos
        throughput_vehiculos_Predefinido{i} = [];
        throughput_vehiculos_Fijo{i} = [];
        throughput_vehiculos_Movil{i} = [];
        bler_movil{i}=[];
        bler_fijo{i}=[];
        bler_predefinido{i}=[];
        simThroughput_vehiculos_Predefinido{i} = [];
        maxThroughput_vehiculos_Predefinido{i} = [];
        simThroughput_vehiculos_Fijo{i} = [];
        maxThroughput_vehiculos_Fijo{i} = [];
        simThroughput_vehiculos_Movil{i} = [];
        maxThroughput_vehiculos_Movil{i} = [];
    end

    % Initializing the communications system
    %% Creación del sistema de comunicación
    simParameters = struct();
    simParameters.NFrames = 1;                                % Número de 10 ms Frames
    simParameters.PerfectChannelEstimator = true;             % Estimación y sincronización del canal perfectas, si es false se basa en la señal DM-RS recibida en el PDSCH
    simParameters.DisplaySimulationInformation = true;        % Muestra info HARQ
    simParameters.DisplayDiagnostics = false;                 % Grafica los gráficos de EVM (error vector magnitude) por capa, muestra EVM de capa por slot (tiempo) y bloque de recursos (frec)
    simParameters.Carrier = nrCarrierConfig;                  % Configuración de la cuadrícula de recursos (nr en este caso)
    simParameters.Carrier.NSizeGrid = 51;                     % BW en bloques de recursos (51 RBs a 30 kHz en el espaciado de subportadoras (SCS) para 20 MHz BW)
    simParameters.Carrier.SubcarrierSpacing = 30;             % 15, 30, 50, 120 kHz
    simParameters.Carrier.CyclicPrefix = 'Normal';            % 'Normal' o 'Extended' (este es importante solo para 60 kHz SCS)
    simParameters.Carrier.NCellID = 1;                        % Identificación de la celda
    simParameters.CarrierFrequency = 5.9e9;                   % Frecuencia de la carrier 700e6 en 36.777
    simParameters.TxHeight = 100;                             % Altura de los drones (m)
    simParameters.TxPower = 5;                               % Potencia transmitida (dBm) En el doc 36.777 dice que son46 dBm
    simParameters.RxHeight = 1.5;                             % Altura del receptor (m)
    simParameters.RxNoiseFigure = 7;                          % Figura de ruido del receptor (dB) 6, 9 el doc
    simParameters.RxAntTemperature = 300;                     % Temperatura de la antena receptora (K)
    simParameters.PathLossModel = '5G-NR';                    % '5G-NR' o 'fspl'
    simParameters.PathLoss = nrPathLossConfig;
    simParameters.PathLoss.Scenario = 'RMa';                  % Rural macrocell
    simParameters.PathLoss.EnvironmentHeight = 0;             % Altura promedio del escenario (m)
    simParameters.DelayProfile = 'CDL-D';                     % A, B, C para NLOS y D, E para LOS channels
    simParameters.MaximumDopplerShift = 5;                    % Máximo desplazamiento doppler
    simParameters.DelaySpread = 300e-9;                       % Dispersión de retardo en segundos
    simParameters.NTxAnts = 1;                                % Número de antenas Tx
    simParameters.NRxAnts = 1;                                % Número de antenas Rx

    % Configuración PDSCH
    simParameters.PDSCH = nrPDSCHConfig;
    simParameters.PDSCHExtension = struct();
    simParameters.PDSCH.PRBSet = 0:simParameters.Carrier.NSizeGrid-1;    % Definición de bloques de recursos físicos (PRB) en PDSCH
    simParameters.PDSCH.SymbolAllocation = [0, simParameters.Carrier.SymbolsPerSlot]; %Asignación de símbolos dentro de un slot
    simParameters.PDSCH.MappingType = 'A';                    % Como se mapean los datos en símbolos y PRB, A para mapeo basado en slots y B no basados en slots
    simParameters.PDSCH.NID = simParameters.Carrier.NCellID;  % Identificadores de scrambling
    simParameters.PDSCH.RNTI = 1;
    simParameters.PDSCH.VRBToPRBInterleaving = 0;             % 0 para desactivar el entrelazado (interleaving) en la asignación de recursos virtuales (VRB) a PRB
    simParameters.PDSCH.VRBBundleSize = 4;                    % Tamaño del paquete VRB (Los VRB se agrupan en bloques de 4 para su mapeo a PRB
    simParameters.PDSCH.NumLayers = 1;                        % Número de capas de transmisión PDSCH

    % Definición de la modulación y tasa de codificación. esto depende
    % directamente del número de capas
    if simParameters.PDSCH.NumCodewords > 1                             % Multicodeword transmission (when number of layers being > 4)
        simParameters.PDSCH.Modulation = {'16QAM','16QAM'};             % 'QPSK', '16QAM', '64QAM', '256QAM'
        simParameters.PDSCHExtension.TargetCodeRate = [490 490]/1024;   % Tasa de codificación objetivo para calcular el tamaño de bloque de transporte
    else
        simParameters.PDSCH.Modulation = '16QAM';                       % 'QPSK', '16QAM', '64QAM', '256QAM'
        simParameters.PDSCHExtension.TargetCodeRate = 490/1024;         % Tasa de codificación objetivo para calcular el tamaño de bloque de transporte
    end

    % Configuración DM - RS y puerto de antena (TS 38.211 Section 7.4.1.1)
    % Señal de referencia de demodulación
    simParameters.PDSCH.DMRS.DMRSPortSet = 0:simParameters.PDSCH.NumLayers-1;  % Puertos DMRS para usar en las capas
    simParameters.PDSCH.DMRS.DMRSTypeAPosition = 2;                     % Solo mapeo A, primer símbolo DMRS en posición (2,3)
    simParameters.PDSCH.DMRS.DMRSLength = 1;                            % Número de símbolos DMRS de carga frontal (1 (símbolo único), 2 (símbolo doble))
    simParameters.PDSCH.DMRS.DMRSAdditionalPosition = 2;                % Símbolos DMRS de posición adicional (max range 0 ...3)
    simParameters.PDSCH.DMRS.DMRSConfigurationType = 2;                 % Typo de configuración DMRS (1 o 2)
    simParameters.PDSCH.DMRS.NumCDMGroupsWithoutData = 1;               % Número de grupos de multiplexación por división de código (CDM) sin datos
    simParameters.PDSCH.DMRS.NIDNSCID = 1;                              % Identificación de scrambling (0 .... 65535)
    simParameters.PDSCH.DMRS.NSCID = 0;                                 % Inicialización de scrambling (0 o 1)

    % Configuración PT-RS (TS 38.211 Section 7.4.1.2) Señal de referencia de
    % seguimiento de fase
    simParameters.PDSCH.EnablePTRS = 0;                                 % Activar o no el PTRS (1 o 0)
    simParameters.PDSCH.PTRS.TimeDensity = 1;                           % densidad de tiempo (1,2,4)
    simParameters.PDSCH.PTRS.FrequencyDensity = 2;                      % Densidad de frecuencia (2,4)
    simParameters.PDSCH.PTRS.REOffset = '00';                           % compensación del elemento de recurso ('00', '01', '10', '11')
    simParameters.PDSCH.PTRS.PTRSPortSet = [];                          % Puerto de antena PT-RS, subconjunto del conjunto de puertos DM-RS. Vacío corresponde al número de puerto DM-RS inferior

    % Reserva de patrones PRB si se requieren
    simParameters.PDSCH.ReservedPRB{1}.SymbolSet = [];                  % Reservado para símbolos PDSCH
    simParameters.PDSCH.ReservedPRB{1}.PRBSet = [];                     % Reservado para PRB de PDSCH
    simParameters.PDSCH.ReservedPRB{1}.Period = [];                     % Periocidad de recursos reservados

    % Agrupación de PDSCH PRB (TS 38.214 Sección 5.1.2.3)
    simParameters.PDSCHExtension.PRGBundleSize = [];                    % 2, 4 o [] para indicar BW

    % Proceso HARQ y parámetros de coincidencia de velocidad/TBS
    simParameters.PDSCHExtension.XOverhead = 6 * simParameters.PDSCH.EnablePTRS;    % Sobrecarga de coincidencia de tasa de PDSCH para TBS (Xoh) en 6 cuando PTRS está habilitado, sino es 0
    simParameters.PDSCHExtension.NHARQProcesses = 16;                               % Número de procesos paralelos HARQ a usar
    simParameters.PDSCHExtension.EnableHARQ = true;                     % Permitir  las retransmisiones para cada proceso, usando secuencia RV [0,2,3,1] (Versión redundantes)

    % Parámetros de codificación LDPC
    simParameters.PDSCHExtension.LDPCDecodingAlgorithm = 'Normalized min-sum';  % 'Belief propagation', 'Layered belief propagation', 'Normalized min-sum', 'Offset min-sum'
    simParameters.PDSCHExtension.MaximumLDPCIterationCount = 6;
    simParameters.DataType = 'single';                                  % Definir el tipo de data para la forma de onda y cuadrícula de recursos

    if contains(simParameters.DelayProfile,'CDL','IgnoreCase',true)
        channel = nrCDLChannel; % Objeto de canal CDL
        % Convierte el número de antenas en disposiciones de paneles de antenas.
        % Si NTxAnts no es uno de los valores (1,2,4,8,16,32,64,128,256,512,1024),
        % su valor se redondea hacia arriba al valor más cercano en el conjunto.
        % Si NRxAnts no es 1 o un número par, su valor se redondea hacia el
        % número par más cercano.
        channel = hArrayGeometry(channel, simParameters.NTxAnts, simParameters.NRxAnts);
        simParameters.NTxAnts = prod(channel.TransmitAntennaArray.Size);
        simParameters.NRxAnts = prod(channel.ReceiveAntennaArray.Size);
    else
        channel = nrTDLChannel; % Objeto de canal TDL
        % Configura la geometría del canal
        channel.NumTransmitAntennas = simParameters.NTxAnts;
        channel.NumReceiveAntennas = simParameters.NRxAnts;
    end

    %Asignación de parámetros del canal de simulación, frec de muestreo de
    %forma de onda al objeto y parámetros de pérdidas de camino (Path loss)
    channel.DelayProfile = simParameters.DelayProfile;
    chInfo = info(channel);

    if contains(simParameters.DelayProfile,'CDL','IgnoreCase',true)
        kFactor = chInfo.KFactorFirstCluster; % dB
    else % TDL
        kFactor = chInfo.KFactorFirstTap; % dB
    end

    simParameters.LOS = kFactor > -Inf;                                  % Determinar el LOS entre Tx y Rx basado en el factor K
    waveformInfo = nrOFDMInfo(simParameters.Carrier);                    % Información sobre la forma de onda de banda base después del paso de la modulación OFMD
    channel.DelaySpread = simParameters.DelaySpread;
    channel.MaximumDopplerShift = simParameters.MaximumDopplerShift;
    channel.SampleRate = waveformInfo.SampleRate;
    maxChDelay = chInfo.MaximumChannelDelay;

    validateNumLayers(simParameters);                                    % Validación de la capa PDSCH contra la geometría del canal

    % Configuración de la secuencia de la versión de redundancia (RV) de HARQ
    if simParameters.PDSCHExtension.EnableHARQ                           % En el reporte de RAN WG1 meeting #91 (R1-1719301) indica que si el rendimiento es prioridad hay que usar [0 2 3 1]
        rvSeq = [0 2 3 1];
    else
        rvSeq = 0;                                                       % Si no hay retransmisiones
    end

    % Creación del sistema de de codificación DL-SCH para llevar a cabo la
    % codificación del canal de transporte
    encodeDLSCH = nrDLSCH;
    encodeDLSCH.MultipleHARQProcesses = true;
    encodeDLSCH.TargetCodeRate = simParameters.PDSCHExtension.TargetCodeRate;

    % Creación del decodificación DL-SCH,
    decodeDLSCH = nrDLSCHDecoder;
    decodeDLSCH.MultipleHARQProcesses = true;
    decodeDLSCH.TargetCodeRate = simParameters.PDSCHExtension.TargetCodeRate;
    decodeDLSCH.LDPCDecodingAlgorithm = simParameters.PDSCHExtension.LDPCDecodingAlgorithm;
    decodeDLSCH.MaximumLDPCIterationCount = simParameters.PDSCHExtension.MaximumLDPCIterationCount;


    % Simulation
    %% Simulación
    inicio_mueve = false;

    while true
        traci.simulationStep();
        carsInSim = traci.vehicle.getIDList();  %Obtiene la lista de ID de los vehículos presentes en la simulación

        if isempty(carsInSim)     %Si ya no quedan vehículos en la simulación
            if length(promedioSNR_Predefinido) > str2double(depart)
                break;     %Finaliza la simulación
            end
        else
            if ~inicio_mueve   %Si hay autos, y el movimiento aún no ha comenzado, lo inicia
                inicio_mueve = true;
            end
        end

        if inicio_mueve   %Si se ha iniciado el movimiento, se obtienen las posiciones de los drones
            if ismember(poiID_Predefinido, traci.poi.getIDList())
                pos2D_dron_Predefinido = traci.poi.getPosition(poiID_Predefinido);
                posX_dron_Predefinido = pos2D_dron_Predefinido(1);
                posY_dron_Predefinido = pos2D_dron_Predefinido(2);

                pos3D_history_Predefinido = [pos3D_history_Predefinido; posX_dron_Predefinido, posY_dron_Predefinido, simParameters.TxHeight];
            end

            if ismember(poiID_Fijo, traci.poi.getIDList())
                pos2D_dron_Fijo = traci.poi.getPosition(poiID_Fijo);
                posX_dron_Fijo = pos2D_dron_Fijo(1);
                posY_dron_Fijo = pos2D_dron_Fijo(2);
            end
            if ismember(poiID_Movil, traci.poi.getIDList())
                pos2D_dron_Movil = traci.poi.getPosition(poiID_Movil);
                posX_dron_Movil = pos2D_dron_Movil(1);
                posY_dron_Movil = pos2D_dron_Movil(2);

                pos3D_history_Movil = [pos3D_history_Movil; posX_dron_Movil, posY_dron_Movil, simParameters.TxHeight];
            end

            % Inicia una matriz para almacenar las posiciones de recepción de
            % los vehículos
            rxPositions = zeros(3, num_autos);
            for i = 1:num_autos
                vehicleID = vehicleIDs{i};
                if ismember(vehicleID, carsInSim)
                    pos2D = traci.vehicle.getPosition(vehicleID);
                    rxPositions(:, i) = [pos2D(1); pos2D(2); simParameters.RxHeight];

                    % Verificar si la celda está vacía y luego agregar la nueva posición
                    if isempty(pos3D_history_vehiculos{i})
                        pos3D_history_vehiculos{i} = [pos2D(1), pos2D(2), simParameters.RxHeight];
                    else
                        pos3D_history_vehiculos{i} = [pos3D_history_vehiculos{i}; pos2D(1), pos2D(2), simParameters.RxHeight];
                    end
                else
                    rxPositions(:, i) = NaN;  % Si no está el vehículo lo agrega como NaN
                end
            end

            if ismember(poiID_Predefinido, traci.poi.getIDList()) % Si está el dron Predefinido
                stepSNRs_Predefinido = NaN(1, size(rxPositions, 2));   % Itera para obtener la posición del receptor
                for i = 1:size(rxPositions, 2)
                    if all(~isnan(rxPositions(:, i)))  %Se verifica que no hayan posiciones NaN del receptor
                        txPosition = [posX_dron_Predefinido; posY_dron_Predefinido; simParameters.TxHeight];
                        rxPosition = rxPositions(:, i);   % Se definen las posiciones de Tx y Rx
                        pathLoss = nrPathLoss(simParameters.PathLoss, simParameters.CarrierFrequency, simParameters.LOS, txPosition, rxPosition);
                        % Se calcula el pathLoss con la distancia entre Tx y Rx
                        kBoltz = physconst('Boltzmann');  % Constante de Boltzman
                        NF = 10^(simParameters.RxNoiseFigure / 10);  % Se crea el factor de ruido adimensional
                        Teq = simParameters.RxAntTemperature + 290 * (NF - 1);  % Ruido de temperatura
                        N0 = sqrt(kBoltz * waveformInfo.SampleRate * Teq / 2.0); %Amplitud del ruido de la antena receptora
                        fftOccupancy = 12 * simParameters.Carrier.NSizeGrid / waveformInfo.Nfft;  %número de puntos de la transformada rápida de Fourier (FFT) utilizada para la modulación OFDM
                        simParameters.SNRIn = (simParameters.TxPower - 30) - pathLoss - 10 * log10(fftOccupancy) - 10 * log10(2 * N0^2); % Obtención del SNR
                        stepSNRs_Predefinido(i) = simParameters.SNRIn;   % Se almacena el SNR
                    end
                end
                SNRs_Predefinido = [SNRs_Predefinido; stepSNRs_Predefinido];
                promedioSNR_Predefinido = [promedioSNR_Predefinido, nanmean(stepSNRs_Predefinido)];
                % Para cada vehículo se procesan los SNRs
                for i = 1:length(vehicleIDs)
                    if ~isnan(stepSNRs_Predefinido(i))
                        SNRdB = stepSNRs_Predefinido(i);
                        % rng('shuffle');   % Se inicia la semilla del generador de números aleatorios
                        % Configuración de las variables del protador, PDSCH y
                        % HARQ
                        carrier = simParameters.Carrier;
                        pdsch = simParameters.PDSCH;
                        pdschextra = simParameters.PDSCHExtension;
                        decodeDLSCHLocal = decodeDLSCH;
                        decodeDLSCHLocal.reset();
                        pathFilters = [];
                        % Número total de slots en la simulación
                        NSlots = simParameters.NFrames * carrier.SlotsPerFrame;
                        % Obtención de la estimación inicial del canal
                        estChannelGridAnts = getInitialChannelEstimate(carrier, simParameters.NTxAnts, channel, simParameters.DataType);
                        % Cálculo de los nuevos pre-codificadores
                        newWtx = hSVDPrecoders(carrier, pdsch, estChannelGridAnts, pdschextra.PRGBundleSize);
                        offset = 0;
                        simThroughput = 0; %Inicio de variables de rendimiento
                        maxThroughput = 0;
                        err_predefinido=0;
                        harqSequence = 0:pdschextra.NHARQProcesses-1;
                        harqEntity = HARQEntity(harqSequence, rvSeq, pdsch.NumCodewords);
                        err_movil = 0;
                        for nslot = 0:NSlots-1 % Se itera sobre cada slot de la simulación
                            carrier.NSlot = nslot; % Configura el slot actual del portador
                            [pdschIndices, pdschIndicesInfo] = nrPDSCHIndices(carrier, pdsch);  % Obtención de indices PDSCH
                            % Cálculo de los tamaños de los bloques de
                            % trasnporte TBS
                            trBlkSizes = nrTBS(pdsch.Modulation, pdsch.NumLayers, numel(pdsch.PRBSet), pdschIndicesInfo.NREPerPRB, pdschextra.TargetCodeRate, pdschextra.XOverhead);
                            % Se itera sobre cada codeword
                            for cwIdx = 1:pdsch.NumCodewords
                                % Si hay nuevos datos para la palabra de código
                                if harqEntity.NewData(cwIdx)
                                    trBlk = randi([0 1], trBlkSizes(cwIdx), 1);% Se genera un TBS aleatorio
                                    setTransportBlock(encodeDLSCH, trBlk, cwIdx-1, harqEntity.HARQProcessID); % Si el temporizador de secuencia de HARQ ha expirado, reinicia el búfer suave
                                    if harqEntity.SequenceTimeout(cwIdx)
                                        resetSoftBuffer(decodeDLSCHLocal, cwIdx-1, harqEntity.HARQProcessID);
                                    end
                                end
                            end
                            % Codifica los bloques de transporte
                            codedTrBlocks = encodeDLSCH(pdsch.Modulation, pdsch.NumLayers, pdschIndicesInfo.G, harqEntity.RedundancyVersion, harqEntity.HARQProcessID);
                            wtx = newWtx; % Pre-codifica las señales PDSCH
                            pdschGrid = nrResourceGrid(carrier, simParameters.NTxAnts, 'OutputDataType', simParameters.DataType);
                            pdschSymbols = nrPDSCH(carrier, pdsch, codedTrBlocks);
                            [pdschAntSymbols, pdschAntIndices] = nrPDSCHPrecode(carrier, pdschSymbols, pdschIndices, wtx);
                            pdschGrid(pdschAntIndices) = pdschAntSymbols;
                            % Genera y pre-codifica las señales DM-RS
                            dmrsSymbols = nrPDSCHDMRS(carrier, pdsch);
                            dmrsIndices = nrPDSCHDMRSIndices(carrier, pdsch);
                            [dmrsAntSymbols, dmrsAntIndices] = nrPDSCHPrecode(carrier, dmrsSymbols, dmrsIndices, wtx);
                            pdschGrid(dmrsAntIndices) = dmrsAntSymbols;
                            % Genera y pre-codifica las señales PT-RS
                            ptrsSymbols = nrPDSCHPTRS(carrier, pdsch);
                            ptrsIndices = nrPDSCHPTRSIndices(carrier, pdsch);
                            [ptrsAntSymbols, ptrsAntIndices] = nrPDSCHPrecode(carrier, ptrsSymbols, ptrsIndices, wtx);
                            pdschGrid(ptrsAntIndices) = ptrsAntSymbols;
                            % Realiza la modulación OFDM
                            txWaveform = nrOFDMModulate(carrier, pdschGrid);
                            txWaveform = [txWaveform; zeros(maxChDelay, size(txWaveform, 2))];
                            % Pasa la señal a través del canal y obtiene la forma de onda recibida
                            [rxWaveform, pathGains, sampleTimes] = channel(txWaveform);
                            SNR = 10^(SNRdB/10); % Calcula el SNR lineal
                             N0 = 1/sqrt(simParameters.NRxAnts * double(waveformInfo.Nfft) * SNR); % Calcula el AWGN
                            noise = N0 * randn(size(rxWaveform), "like", rxWaveform);
                            rxWaveform = rxWaveform + noise;
                            % Estimación perfecta del canal
                            if simParameters.PerfectChannelEstimator
                                pathFilters = getPathFilters(channel);
                                [offset, mag] = nrPerfectTimingEstimate(pathGains, pathFilters);
                            else % Estimación práctica del canal
                                [t, mag] = nrTimingEstimate(carrier, rxWaveform, dmrsIndices, dmrsSymbols);
                                offset = hSkipWeakTimingOffset(offset, t, mag);
                                if offset > maxChDelay
                                    warning(['Estimated timing offset (%d) is greater than the maximum channel delay (%d).' ...
                                        ' This will result in a decoding failure. This may be caused by low SNR,' ...
                                        ' or not enough DM-RS symbols to synchronize successfully.'], offset, maxChDelay);
                                end
                            end
                            % Ajusta la señal recibida de acuerdo con el desfase estimado
                            rxWaveform = rxWaveform(1 + offset:end, :);
                            % Realiza la demodulación OFDM de la señal recibida
                            rxGrid = nrOFDMDemodulate(carrier, rxWaveform);
                            [K, L, R] = size(rxGrid);
                            % Si el número de símbolos en la rejilla es menor que los símbolos por slot, completa con ceros
                            if L < carrier.SymbolsPerSlot
                                rxGrid = cat(2, rxGrid, zeros(K, carrier.SymbolsPerSlot - L, R));
                            end
                            % Estimación del canal utilizando la estimación perfecta si está habilitada
                            if simParameters.PerfectChannelEstimator
                                estChannelGridAnts = nrPerfectChannelEstimate(carrier, pathGains, pathFilters, offset, sampleTimes);
                                noiseGrid = nrOFDMDemodulate(carrier, noise(1 + offset:end, :));
                                noiseEst = var(noiseGrid(:));
                                % Extrae los recursos PDSCH y la estimación del canal
                                [pdschRx, pdschHest, ~, pdschHestIndices] = nrExtractResources(pdschIndices, rxGrid, estChannelGridAnts);
                                pdschHest = nrPDSCHPrecode(carrier, pdschHest, pdschHestIndices, permute(wtx, [2 1 3]));
                            else % Estimación práctica del canal usando subbandas
                                [estChannelGridPorts, noiseEst] = hSubbandChannelEstimate(carrier, rxGrid, dmrsIndices, dmrsSymbols, pdschextra.PRGBundleSize, 'CDMLengths', pdsch.DMRS.CDMLengths);
                                noiseEst = mean(noiseEst, 'all');
                                [pdschRx, pdschHest] = nrExtractResources(pdschIndices, rxGrid, estChannelGridPorts);
                                estChannelGridAnts = precodeChannelEstimate(carrier, estChannelGridPorts, conj(wtx));
                            end
                            % Igualación MMSE de las señales PDSCH (Minimum Mean Square Error)
                            [pdschEq, csi] = nrEqualizeMMSE(pdschRx, pdschHest, noiseEst);
                            if ~isempty(ptrsIndices) % Si existen PTRS, realiza la compensación de fase común (CPE)
                                tempGrid = nrResourceGrid(carrier, pdsch.NumLayers);
                                [ptrsRx, ptrsHest, ~, ~, ptrsHestIndices, ptrsLayerIndices] = nrExtractResources(ptrsIndices, rxGrid, estChannelGridAnts, tempGrid);
                                ptrsHest = nrPDSCHPrecode(carrier, ptrsHest, ptrsHestIndices, permute(wtx, [2 1 3]));
                                ptrsEq = nrEqualizeMMSE(ptrsRx, ptrsHest, noiseEst);
                                tempGrid(ptrsLayerIndices) = ptrsEq;
                                cpe = nrChannelEstimate(tempGrid, ptrsIndices, ptrsSymbols); % Estimación de la fase común (CPE)
                                cpe = angle(sum(cpe, [1 3 4]));
                                tempGrid(pdschIndices) = pdschEq;
                                symLoc = pdschIndicesInfo.PTRSSymbolSet(1) + 1:pdschIndicesInfo.PTRSSymbolSet(end) + 1;
                                tempGrid(:, symLoc, :) = tempGrid(:, symLoc, :) .* exp(-1i * cpe(symLoc));
                                pdschEq = tempGrid(pdschIndices);
                            end
                            % Decodificación PDSCH y obtención de los LLRs del DLSCH
                            [dlschLLRs, rxSymbols] = nrPDSCHDecode(carrier, pdsch, pdschEq, noiseEst);
                            if simParameters.DisplayDiagnostics % Si la visualización de diagnósticos está habilitada, muestra el EVM por capa
                                plotLayerEVM(NSlots, nslot, pdsch, size(pdschGrid), pdschIndices, pdschSymbols, pdschEq);
                            end
                            csi = nrLayerDemap(csi); % Desempaqueta el CSI y aplica la ponderación a los LLRs del DLSCH
                            for cwIdx = 1:pdsch.NumCodewords
                                Qm = length(dlschLLRs{cwIdx}) / length(rxSymbols{cwIdx});
                                csi{cwIdx} = repmat(csi{cwIdx}.', Qm, 1);
                                dlschLLRs{cwIdx} = dlschLLRs{cwIdx} .* csi{cwIdx}(:);
                            end
                            decodeDLSCHLocal.TransportBlockLength = trBlkSizes; % Configura la longitud del bloque de transporte y decodifica el DLSCH
                            [decbits, blkerr] = decodeDLSCHLocal(dlschLLRs, pdsch.Modulation, pdsch.NumLayers, harqEntity.RedundancyVersion, harqEntity.HARQProcessID);
                            % Actualiza el throughput simulado y el throughput máximo posible
                            simThroughput = simThroughput + sum(~blkerr .* trBlkSizes);
                            maxThroughput = maxThroughput + sum(trBlkSizes);
                            err_predefinido = err_predefinido + (~isequal(decbits,trBlk));
                            % Actualiza el estado del proceso HARQ y muestra información de la simulación
                            procstatus = updateAndAdvance(harqEntity, blkerr, trBlkSizes, pdschIndicesInfo.G);
                            % if simParameters.DisplaySimulationInformation
                            %     fprintf('\n(%3.2f%%) NSlot=%d, %s', 100 * (nslot + 1) / NSlots, nslot, procstatus);
                            % end
                            % Recalcula los pre-codificadores para el siguiente slot
                            newWtx = hSVDPrecoders(carrier, pdsch, estChannelGridAnts, pdschextra.PRGBundleSize);
                        end
                        % Calcula el porcentaje de throughput alcanzado y lo almacena
                        throughputPercentage = simThroughput * 100 / maxThroughput;
                        throughput_vehiculos_Predefinido{i} = [throughput_vehiculos_Predefinido{i}; throughputPercentage];
                        simThroughput_vehiculos_Predefinido{i} = [simThroughput_vehiculos_Predefinido{i}; simThroughput];
                        maxThroughput_vehiculos_Predefinido{i} = [maxThroughput_vehiculos_Predefinido{i}; maxThroughput];
                        bler_predefinido1 = err_predefinido/NSlots;
                        bler_predefinido{i} = [bler_predefinido{i}; bler_predefinido1];
                    end
                end
            end

            if ismember(poiID_Fijo, traci.poi.getIDList()) % Si está el dron Fijo
                stepSNRs_Fijo = NaN(1, size(rxPositions, 2)); % Se inicializa un vector para almacenar los SNRs de cada receptor
                for i = 1:size(rxPositions, 2)
                    if all(~isnan(rxPositions(:, i))) % Se verifica que no hayan posiciones NaN del receptor
                        txPosition = [posX_dron_Fijo; posY_dron_Fijo; simParameters.TxHeight];
                        rxPosition = rxPositions(:, i); % Se definen las posiciones de Tx y Rx
                        pathLoss = nrPathLoss(simParameters.PathLoss, simParameters.CarrierFrequency, simParameters.LOS, txPosition, rxPosition); % Se calcula el path loss
                        kBoltz = physconst('Boltzmann'); % Constante de Boltzmann
                        NF = 10^(simParameters.RxNoiseFigure / 10); % Se crea el factor de ruido adimensional
                        Teq = simParameters.RxAntTemperature + 290 * (NF - 1); % Ruido de temperatura
                        N0 = sqrt(kBoltz * waveformInfo.SampleRate * Teq / 2.0); % Amplitud del ruido de la antena receptora
                        fftOccupancy = 12 * simParameters.Carrier.NSizeGrid / waveformInfo.Nfft; % Número de puntos de la transformada rápida de Fourier (FFT) utilizada para la modulación OFDM
                        simParameters.SNRIn = (simParameters.TxPower - 30) - pathLoss - 10 * log10(fftOccupancy) - 10 * log10(2 * N0^2); % Obtención del SNR
                        stepSNRs_Fijo(i) = simParameters.SNRIn; % Se almacena el SNR para este receptor
                    end
                end
                SNRs_Fijo = [SNRs_Fijo; stepSNRs_Fijo]; % Se agrega la lista de SNRs a la lista global de SNRs para el dron fijo
                promedioSNR_Fijo = [promedioSNR_Fijo, mean(stepSNRs_Fijo)]; % Se calcula y almacena el SNR promedio para este step
                % Para cada vehículo se procesan los SNRs
                for i = 1:length(vehicleIDs)
                    if ~isnan(stepSNRs_Fijo(i)) % Si el SNR para este vehículo no es NaN
                        SNRdB = stepSNRs_Fijo(i); % Se asigna el SNR en decibelios
                      %  rng('shuffle'); % Se inicializa la semilla del generador de números aleatorios
                        % Configuración de las variables del portador, PDSCH y HARQ
                        carrier = simParameters.Carrier;
                        pdsch = simParameters.PDSCH;
                        pdschextra = simParameters.PDSCHExtension;
                        decodeDLSCHLocal = decodeDLSCH;
                        decodeDLSCHLocal.reset();
                        pathFilters = [];
                        % Número total de slots en la simulación
                        NSlots = simParameters.NFrames * carrier.SlotsPerFrame;
                        % Obtención de la estimación inicial del canal
                        estChannelGridAnts = getInitialChannelEstimate(carrier, simParameters.NTxAnts, channel, simParameters.DataType);
                        % Cálculo de los nuevos pre-codificadores
                        newWtx = hSVDPrecoders(carrier, pdsch, estChannelGridAnts, pdschextra.PRGBundleSize);
                        offset = 0;
                        simThroughput = 0; % Inicialización de variables de rendimiento
                        maxThroughput = 0;
                        err_fijo=0;
                        harqSequence = 0:pdschextra.NHARQProcesses-1;
                        harqEntity = HARQEntity(harqSequence, rvSeq, pdsch.NumCodewords);
                        for nslot = 0:NSlots-1 % Se itera sobre cada slot de la simulación
                            carrier.NSlot = nslot; % Configura el slot actual del portador
                            [pdschIndices, pdschIndicesInfo] = nrPDSCHIndices(carrier, pdsch); % Obtención de índices PDSCH
                            % Cálculo de los tamaños de los bloques de transporte (TBS)
                            trBlkSizes = nrTBS(pdsch.Modulation, pdsch.NumLayers, numel(pdsch.PRBSet), pdschIndicesInfo.NREPerPRB, pdschextra.TargetCodeRate, pdschextra.XOverhead);
                            % Se itera sobre cada codeword
                            for cwIdx = 1:pdsch.NumCodewords
                                % Si hay nuevos datos para la palabra de código
                                if harqEntity.NewData(cwIdx)
                                    trBlk = randi([0 1], trBlkSizes(cwIdx), 1); % Se genera un TBS aleatorio
                                    setTransportBlock(encodeDLSCH, trBlk, cwIdx-1, harqEntity.HARQProcessID); % Se asigna el bloque de transporte al codificador
                                    if harqEntity.SequenceTimeout(cwIdx) % Si el temporizador de secuencia de HARQ ha expirado, reinicia el búfer suave
                                        resetSoftBuffer(decodeDLSCHLocal, cwIdx-1, harqEntity.HARQProcessID);
                                    end
                                end
                            end
                            % Codifica los bloques de transporte
                            codedTrBlocks = encodeDLSCH(pdsch.Modulation, pdsch.NumLayers, pdschIndicesInfo.G, harqEntity.RedundancyVersion, harqEntity.HARQProcessID);
                            wtx = newWtx; % Asigna los pre-codificadores calculados
                            % Se crea una rejilla de recursos para almacenar los símbolos PDSCH
                            pdschGrid = nrResourceGrid(carrier, simParameters.NTxAnts, 'OutputDataType', simParameters.DataType);
                            % Modula los bloques de transporte codificados en símbolos PDSCH
                            pdschSymbols = nrPDSCH(carrier, pdsch, codedTrBlocks);
                            [pdschAntSymbols, pdschAntIndices] = nrPDSCHPrecode(carrier, pdschSymbols, pdschIndices, wtx); % Precodifica los símbolos PDSCH
                            pdschGrid(pdschAntIndices) = pdschAntSymbols;
                            % Genera y pre-codifica las señales DM-RS
                            dmrsSymbols = nrPDSCHDMRS(carrier, pdsch);
                            dmrsIndices = nrPDSCHDMRSIndices(carrier, pdsch);
                            [dmrsAntSymbols, dmrsAntIndices] = nrPDSCHPrecode(carrier, dmrsSymbols, dmrsIndices, wtx);
                            pdschGrid(dmrsAntIndices) = dmrsAntSymbols;
                            % Genera y pre-codifica las señales PT-RS
                            ptrsSymbols = nrPDSCHPTRS(carrier, pdsch);
                            ptrsIndices = nrPDSCHPTRSIndices(carrier, pdsch);
                            [ptrsAntSymbols, ptrsAntIndices] = nrPDSCHPrecode(carrier, ptrsSymbols, ptrsIndices, wtx);
                            pdschGrid(ptrsAntIndices) = ptrsAntSymbols;
                            % Realiza la modulación OFDM
                            txWaveform = nrOFDMModulate(carrier, pdschGrid);
                            txWaveform = [txWaveform; zeros(maxChDelay, size(txWaveform, 2))];
                            % Pasa la señal a través del canal y obtiene la forma de onda recibida
                            [rxWaveform, pathGains, sampleTimes] = channel(txWaveform);
                            SNR = 10^(SNRdB / 10); % Calcula el SNR lineal
                             N0 = 1 / sqrt(simParameters.NRxAnts * double(waveformInfo.Nfft) * SNR); % Calcula el AWGN
                            noise = N0 * randn(size(rxWaveform), "like", rxWaveform); % Añade ruido a la señal recibida
                            rxWaveform = rxWaveform + noise;
                            % Estimación perfecta del canal
                            if simParameters.PerfectChannelEstimator
                                pathFilters = getPathFilters(channel);
                                [offset, mag] = nrPerfectTimingEstimate(pathGains, pathFilters); % Realiza una estimación de la sincronización
                            else % Estimación práctica del canal
                                [t, mag] = nrTimingEstimate(carrier, rxWaveform, dmrsIndices, dmrsSymbols);
                                offset = hSkipWeakTimingOffset(offset, t, mag);
                                if offset > maxChDelay
                                    warning(['Estimated timing offset (%d) is greater than the maximum channel delay (%d).' ...
                                        ' This will result in a decoding failure. This may be caused by low SNR,' ...
                                        ' or not enough DM-RS symbols to synchronize successfully.'], offset, maxChDelay);
                                end
                            end
                            % Ajusta la señal recibida de acuerdo con el desfase estimado
                            rxWaveform = rxWaveform(1 + offset:end, :);
                            % Realiza la demodulación OFDM de la señal recibida
                            rxGrid = nrOFDMDemodulate(carrier, rxWaveform);
                            [K, L, R] = size(rxGrid);

                            % Si el número de símbolos en la rejilla es menor que los símbolos por slot, completa con ceros
                            if L < carrier.SymbolsPerSlot
                                rxGrid = cat(2, rxGrid, zeros(K, carrier.SymbolsPerSlot - L, R));
                            end
                            % Estimación del canal utilizando la estimación perfecta si está habilitada
                            if simParameters.PerfectChannelEstimator
                                estChannelGridAnts = nrPerfectChannelEstimate(carrier, pathGains, pathFilters, offset, sampleTimes);
                                noiseGrid = nrOFDMDemodulate(carrier, noise(1 + offset:end, :));
                                noiseEst = var(noiseGrid(:));
                                % Extrae los recursos PDSCH y la estimación del canal
                                [pdschRx, pdschHest, ~, pdschHestIndices] = nrExtractResources(pdschIndices, rxGrid, estChannelGridAnts);
                                pdschHest = nrPDSCHPrecode(carrier, pdschHest, pdschHestIndices, permute(wtx, [2 1 3]));
                            else % Estimación práctica del canal usando subbandas
                                [estChannelGridPorts, noiseEst] = hSubbandChannelEstimate(carrier, rxGrid, dmrsIndices, dmrsSymbols, pdschextra.PRGBundleSize, 'CDMLengths', pdsch.DMRS.CDMLengths);
                                noiseEst = mean(noiseEst, 'all');
                                [pdschRx, pdschHest] = nrExtractResources(pdschIndices, rxGrid, estChannelGridPorts);
                                estChannelGridAnts = precodeChannelEstimate(carrier, estChannelGridPorts, conj(wtx));
                            end
                            % Igualación MMSE de las señales PDSCH (Minimum Mean Square Error)
                            [pdschEq, csi] = nrEqualizeMMSE(pdschRx, pdschHest, noiseEst);
                            if ~isempty(ptrsIndices) % Si existen PTRS, realiza la compensación de fase común (CPE)
                                tempGrid = nrResourceGrid(carrier, pdsch.NumLayers);
                                [ptrsRx, ptrsHest, ~, ~, ptrsHestIndices, ptrsLayerIndices] = nrExtractResources(ptrsIndices, rxGrid, estChannelGridAnts, tempGrid);
                                ptrsHest = nrPDSCHPrecode(carrier, ptrsHest, ptrsHestIndices, permute(wtx, [2 1 3]));
                                ptrsEq = nrEqualizeMMSE(ptrsRx, ptrsHest, noiseEst);
                                tempGrid(ptrsLayerIndices) = ptrsEq;
                                cpe = nrChannelEstimate(tempGrid, ptrsIndices, ptrsSymbols); % Estimación de la fase común (CPE)
                                cpe = angle(sum(cpe, [1 3 4]));
                                tempGrid(pdschIndices) = pdschEq;
                                symLoc = pdschIndicesInfo.PTRSSymbolSet(1) + 1:pdschIndicesInfo.PTRSSymbolSet(end) + 1;
                                tempGrid(:, symLoc, :) = tempGrid(:, symLoc, :) .* exp(-1i * cpe(symLoc));
                                pdschEq = tempGrid(pdschIndices);
                            end
                            % Decodificación PDSCH y obtención de los LLRs del DLSCH
                            [dlschLLRs, rxSymbols] = nrPDSCHDecode(carrier, pdsch, pdschEq, noiseEst);
                            if simParameters.DisplayDiagnostics % Si la visualización de diagnósticos está habilitada, muestra el EVM por capa
                                plotLayerEVM(NSlots, nslot, pdsch, size(pdschGrid), pdschIndices, pdschSymbols, pdschEq);
                            end
                            csi = nrLayerDemap(csi); % Desempaqueta el CSI y aplica la ponderación a los LLRs del DLSCH
                            for cwIdx = 1:pdsch.NumCodewords
                                Qm = length(dlschLLRs{cwIdx}) / length(rxSymbols{cwIdx});
                                csi{cwIdx} = repmat(csi{cwIdx}.', Qm, 1);
                                dlschLLRs{cwIdx} = dlschLLRs{cwIdx} .* csi{cwIdx}(:);
                            end
                            decodeDLSCHLocal.TransportBlockLength = trBlkSizes; % Configura la longitud del bloque de transporte y decodifica el DLSCH
                            [decbits, blkerr] = decodeDLSCHLocal(dlschLLRs, pdsch.Modulation, pdsch.NumLayers, harqEntity.RedundancyVersion, harqEntity.HARQProcessID);
                            % Actualiza el throughput simulado y el throughput máximo posible
                            simThroughput = simThroughput + sum(~blkerr .* trBlkSizes);
                            maxThroughput = maxThroughput + sum(trBlkSizes);
                            err_fijo = err_fijo + (~isequal(decbits,trBlk));
                            % Actualiza el estado del proceso HARQ y muestra información de la simulación
                            procstatus = updateAndAdvance(harqEntity, blkerr, trBlkSizes, pdschIndicesInfo.G);
                            % if simParameters.DisplaySimulationInformation
                            %     fprintf('\n(%3.2f%%) NSlot=%d, %s', 100 * (nslot + 1) / NSlots, nslot, procstatus);
                            % end
                            % Recalcula los pre-codificadores para el siguiente slot
                            newWtx = hSVDPrecoders(carrier, pdsch, estChannelGridAnts, pdschextra.PRGBundleSize);
                        end
                        % Calcula el porcentaje de throughput alcanzado y lo almacena
                        throughputPercentage = simThroughput * 100 / maxThroughput;
                        throughput_vehiculos_Fijo{i} = [throughput_vehiculos_Fijo{i}; throughputPercentage];
                        simThroughput_vehiculos_Fijo{i} = [simThroughput_vehiculos_Fijo{i}; simThroughput];
                        maxThroughput_vehiculos_Fijo{i} = [maxThroughput_vehiculos_Fijo{i}; maxThroughput];
                        bler_fijo1 = err_fijo/NSlots;
                        bler_fijo{i} = [bler_fijo{i}; bler_fijo1];
                    end
                end
                if ismember(poiID_Movil, traci.poi.getIDList())  % Si está el dron móvil
                    stepSNRs_Movil = NaN(1, size(rxPositions, 2));  % Inicializa el array para almacenar los valores de SNR
                    for i = 1:size(rxPositions, 2)
                        if all(~isnan(rxPositions(:, i)))  % Verifica que no haya posiciones NaN en los receptores
                            txPosition = [posX_dron_Movil; posY_dron_Movil; simParameters.TxHeight];  % Define la posición del transmisor
                            rxPosition = rxPositions(:, i);  % Define la posición del receptor
                            % Calcula la pérdida de trayectoria (path loss) usando la configuración del simulador
                            pathLoss = nrPathLoss(simParameters.PathLoss, simParameters.CarrierFrequency, simParameters.LOS, txPosition, rxPosition);
                            kBoltz = physconst('Boltzmann');  % Constante de Boltzmann
                            NF = 10^(simParameters.RxNoiseFigure / 10);  % Factor de ruido adimensional
                            Teq = simParameters.RxAntTemperature + 290 * (NF - 1);  % Ruido de temperatura
                            % Amplitud del ruido de la antena receptora
                            N0 = sqrt(kBoltz * waveformInfo.SampleRate * Teq / 2.0);
                            % Ocupación de FFT basada en el tamaño de la cuadrícula de recursos OFDM
                            fftOccupancy = 12 * simParameters.Carrier.NSizeGrid / waveformInfo.Nfft;
                            % Cálculo del SNR resultante
                            simParameters.SNRIn = (simParameters.TxPower - 30) - pathLoss - 10 * log10(fftOccupancy) - 10 * log10(2 * N0^2);
                            stepSNRs_Movil(i) = simParameters.SNRIn;  % Almacena el valor del SNR calculado
                        end
                    end

                    % Almacena los SNRs obtenidos y calcula el promedio
                    SNRs_Movil = [SNRs_Movil; stepSNRs_Movil];
                    promedioSNR_Movil = [promedioSNR_Movil, nanmean(stepSNRs_Movil)];

                    % Procesa los SNRs para cada vehículo
                    for i = 1:length(vehicleIDs)
                        if ~isnan(stepSNRs_Movil(i))
                            SNRdB = stepSNRs_Movil(i);  % SNR en dB
                           % rng('shuffle');  % Inicia la semilla del generador de números aleatorios
                            % Configuración de las variables del portador, PDSCH y HARQ
                            carrier = simParameters.Carrier;
                            pdsch = simParameters.PDSCH;
                            pdschextra = simParameters.PDSCHExtension;
                            decodeDLSCHLocal = decodeDLSCH;
                            decodeDLSCHLocal.reset();
                            pathFilters = [];
                            % Número total de slots en la simulación
                            NSlots = simParameters.NFrames * carrier.SlotsPerFrame;
                            % Obtención de la estimación inicial del canal
                            estChannelGridAnts = getInitialChannelEstimate(carrier, simParameters.NTxAnts, channel, simParameters.DataType);
                            % Cálculo de los nuevos pre-codificadores
                            newWtx = hSVDPrecoders(carrier, pdsch, estChannelGridAnts, pdschextra.PRGBundleSize);
                            offset = 0;
                            simThroughput = 0;  % Inicializa el throughput simulado
                            maxThroughput = 0;
                            err_movil = 0;
                            harqSequence = 0:pdschextra.NHARQProcesses-1;
                            harqEntity = HARQEntity(harqSequence, rvSeq, pdsch.NumCodewords);
                            % Itera sobre cada slot de la simulación
                            for nslot = 0:NSlots-1
                                carrier.NSlot = nslot;  % Configura el slot actual del portador
                                % Obtención de índices PDSCH
                                [pdschIndices, pdschIndicesInfo] = nrPDSCHIndices(carrier, pdsch);
                                % Cálculo de los tamaños de los bloques de transporte (TBS)
                                trBlkSizes = nrTBS(pdsch.Modulation, pdsch.NumLayers, numel(pdsch.PRBSet), pdschIndicesInfo.NREPerPRB, pdschextra.TargetCodeRate, pdschextra.XOverhead);

                                % Itera sobre cada codeword
                                for cwIdx = 1:pdsch.NumCodewords
                                    % Si hay nuevos datos para la codeword
                                    if harqEntity.NewData(cwIdx)
                                        % Genera un TBS aleatorio
                                        trBlk = randi([0 1], trBlkSizes(cwIdx), 1);
                                        % Configura el bloque de transporte en el codificador DLSCH
                                        setTransportBlock(encodeDLSCH, trBlk, cwIdx-1, harqEntity.HARQProcessID);
                                        % Si el temporizador de HARQ ha expirado, reinicia el buffer suave
                                        if harqEntity.SequenceTimeout(cwIdx)
                                            resetSoftBuffer(decodeDLSCHLocal, cwIdx-1, harqEntity.HARQProcessID);
                                        end
                                    end
                                end
                                % Codifica los bloques de transporte
                                codedTrBlocks = encodeDLSCH(pdsch.Modulation, pdsch.NumLayers, pdschIndicesInfo.G, harqEntity.RedundancyVersion, harqEntity.HARQProcessID);
                                wtx = newWtx;  % Pre-codifica las señales PDSCH
                                pdschGrid = nrResourceGrid(carrier, simParameters.NTxAnts, 'OutputDataType', simParameters.DataType);
                                pdschSymbols = nrPDSCH(carrier, pdsch, codedTrBlocks);
                                % Pre-codifica y asigna las señales a la cuadrícula PDSCH
                                [pdschAntSymbols, pdschAntIndices] = nrPDSCHPrecode(carrier, pdschSymbols, pdschIndices, wtx);
                                pdschGrid(pdschAntIndices) = pdschAntSymbols;
                                % Genera y pre-codifica las señales DM-RS
                                dmrsSymbols = nrPDSCHDMRS(carrier, pdsch);
                                dmrsIndices = nrPDSCHDMRSIndices(carrier, pdsch);
                                [dmrsAntSymbols, dmrsAntIndices] = nrPDSCHPrecode(carrier, dmrsSymbols, dmrsIndices, wtx);
                                pdschGrid(dmrsAntIndices) = dmrsAntSymbols;
                                % Genera y pre-codifica las señales PT-RS
                                ptrsSymbols = nrPDSCHPTRS(carrier, pdsch);
                                ptrsIndices = nrPDSCHPTRSIndices(carrier, pdsch);
                                [ptrsAntSymbols, ptrsAntIndices] = nrPDSCHPrecode(carrier, ptrsSymbols, ptrsIndices, wtx);
                                pdschGrid(ptrsAntIndices) = ptrsAntSymbols;
                                % Realiza la modulación OFDM
                                txWaveform = nrOFDMModulate(carrier, pdschGrid);
                                txWaveform = [txWaveform; zeros(maxChDelay, size(txWaveform, 2))];
                                % Pasa la señal a través del canal y obtiene la forma de onda recibida
                                [rxWaveform, pathGains, sampleTimes] = channel(txWaveform);
                                SNR = 10^(SNRdB / 10);  % Convierte el SNR de dB a lineal
                                % Calcula el ruido AWGN
                                 N0 = 1 / sqrt(simParameters.NRxAnts * double(waveformInfo.Nfft) * SNR);
                                noise = N0 * randn(size(rxWaveform), "like", rxWaveform);
                                rxWaveform = rxWaveform + noise;
                                % Estimación perfecta del canal
                                if simParameters.PerfectChannelEstimator
                                    pathFilters = getPathFilters(channel);
                                    [offset, mag] = nrPerfectTimingEstimate(pathGains, pathFilters);
                                else  % Estimación práctica del canal
                                    [t, mag] = nrTimingEstimate(carrier, rxWaveform, dmrsIndices, dmrsSymbols);
                                    offset = hSkipWeakTimingOffset(offset, t, mag);
                                    if offset > maxChDelay
                                        warning(['Estimated timing offset (%d) is greater than the maximum channel delay (%d).' ...
                                            ' This will result in a decoding failure. This may be caused by low SNR,' ...
                                            ' or not enough DM-RS symbols to synchronize successfully.'], offset, maxChDelay);
                                    end
                                end
                                % Ajusta la señal recibida según el desfase estimado
                                rxWaveform = rxWaveform(1 + offset:end, :);
                                % Realiza la demodulación OFDM de la señal recibida
                                rxGrid = nrOFDMDemodulate(carrier, rxWaveform);
                                % Obtiene las dimensiones de la cuadrícula de recursos recibida
                                [K, L, R] = size(rxGrid);
                                % Si el número de símbolos en la cuadrícula es menor que los símbolos por slot, completa con ceros
                                if L < carrier.SymbolsPerSlot
                                    rxGrid = cat(2, rxGrid, zeros(K, carrier.SymbolsPerSlot - L, R));
                                end

                                % Estimación del canal utilizando la estimación perfecta si está habilitada
                                if simParameters.PerfectChannelEstimator
                                    estChannelGridAnts = nrPerfectChannelEstimate(carrier, pathGains, pathFilters, offset, sampleTimes);
                                    noiseGrid = nrOFDMDemodulate(carrier, noise(1 + offset:end, :));
                                    noiseEst = var(noiseGrid(:));
                                    % Extrae los recursos PDSCH y la estimación del canal
                                    [pdschRx, pdschHest, ~, pdschHestIndices] = nrExtractResources(pdschIndices, rxGrid, estChannelGridAnts);
                                    pdschHest = nrPDSCHPrecode(carrier, pdschHest, pdschHestIndices, permute(wtx, [2 1 3]));
                                else  % Estimación práctica del canal usando subbandas
                                    [estChannelGridPorts, noiseEst] = hSubbandChannelEstimate(carrier, rxGrid, dmrsIndices, dmrsSymbols, pdschextra.PRGBundleSize, 'CDMLengths', pdsch.DMRS.CDMLengths);
                                    noiseEst = mean(noiseEst, 'all');
                                    [pdschRx, pdschHest] = nrExtractResources(pdschIndices, rxGrid, estChannelGridPorts);
                                    estChannelGridAnts = precodeChannelEstimate(carrier, estChannelGridPorts, conj(wtx));
                                end

                                % Igualación MMSE de las señales PDSCH (Minimum Mean Square Error)
                                [pdschEq, csi] = nrEqualizeMMSE(pdschRx, pdschHest, noiseEst);
                                % Compensación de fase común (CPE) si existen PTRS
                                if ~isempty(ptrsIndices)
                                    tempGrid = nrResourceGrid(carrier, pdsch.NumLayers);
                                    [ptrsRx, ptrsHest, ~, ~, ptrsHestIndices, ptrsLayerIndices] = nrExtractResources(ptrsIndices, rxGrid, estChannelGridAnts, tempGrid);
                                    ptrsHest = nrPDSCHPrecode(carrier, ptrsHest, ptrsHestIndices, permute(wtx, [2 1 3]));
                                    ptrsEq = nrEqualizeMMSE(ptrsRx, ptrsHest, noiseEst);
                                    tempGrid(ptrsLayerIndices) = ptrsEq;
                                    cpe = nrChannelEstimate(tempGrid, ptrsIndices, ptrsSymbols);  % Estimación de la fase común (CPE)
                                    cpe = angle(sum(cpe, [1 3 4]));
                                    tempGrid(pdschIndices) = pdschEq;
                                    symLoc = pdschIndicesInfo.PTRSSymbolSet(1) + 1:pdschIndicesInfo.PTRSSymbolSet(end) + 1;
                                    tempGrid(:, symLoc, :) = tempGrid(:, symLoc, :) .* exp(-1i * cpe(symLoc));
                                    pdschEq = tempGrid(pdschIndices);
                                end
                                % Decodificación PDSCH y obtención de los LLRs del DLSCH
                                [dlschLLRs, rxSymbols] = nrPDSCHDecode(carrier, pdsch, pdschEq, noiseEst);
                                % Visualización de diagnósticos: Muestra el EVM por capa si está habilitado
                                if simParameters.DisplayDiagnostics
                                    plotLayerEVM(NSlots, nslot, pdsch, size(pdschGrid), pdschIndices, pdschSymbols, pdschEq);
                                end
                                % Desempaqueta el CSI y aplica la ponderación a los LLRs del DLSCH
                                csi = nrLayerDemap(csi);
                                for cwIdx = 1:pdsch.NumCodewords
                                    Qm = length(dlschLLRs{cwIdx}) / length(rxSymbols{cwIdx});
                                    csi{cwIdx} = repmat(csi{cwIdx}.', Qm, 1);
                                    dlschLLRs{cwIdx} = dlschLLRs{cwIdx} .* csi{cwIdx}(:);
                                end
                                % Configura la longitud del bloque de transporte y decodifica el DLSCH
                                decodeDLSCHLocal.TransportBlockLength = trBlkSizes;
                                [decbits, blkerr] = decodeDLSCHLocal(dlschLLRs, pdsch.Modulation, pdsch.NumLayers, harqEntity.RedundancyVersion, harqEntity.HARQProcessID);
                                % Actualiza el throughput simulado y el throughput máximo posible
                                simThroughput = simThroughput + sum(~blkerr .* trBlkSizes);
                                maxThroughput = maxThroughput + sum(trBlkSizes);
                                err_movil = err_movil + (~isequal(decbits,trBlk));
                                % Actualiza el estado del proceso HARQ y muestra información de la simulación
                                procstatus = updateAndAdvance(harqEntity, blkerr, trBlkSizes, pdschIndicesInfo.G);
                                % if simParameters.DisplaySimulationInformation
                                %     fprintf('\n(%3.2f%%) NSlot=%d, %s', 100 * (nslot + 1) / NSlots, nslot, procstatus);
                                % end
                                % Recalcula los pre-codificadores para el siguiente slot
                                newWtx = hSVDPrecoders(carrier, pdsch, estChannelGridAnts, pdschextra.PRGBundleSize);
                            end
                            % Calcula el porcentaje de throughput alcanzado y lo almacena
                            throughputPercentage = simThroughput * 100 / maxThroughput;
                            throughput_vehiculos_Movil{i} = [throughput_vehiculos_Movil{i}; throughputPercentage];
                            simThroughput_vehiculos_Movil{i} = [simThroughput_vehiculos_Movil{i}; simThroughput];
                            maxThroughput_vehiculos_Movil{i} = [maxThroughput_vehiculos_Movil{i}; maxThroughput];
                            bler_movil1 = err_movil/NSlots;
                            bler_movil{i} = [bler_movil{i}; bler_movil1];
                        end
                    end
                end
                %% Movimiento del dron predefinido
                siguiente_posicion = posicion_actual + direccion;
                % Verificar si llegó al final de la secuencia de posiciones
                if siguiente_posicion > size(posiciones, 1)
                    direccion = -1; % Invertir dirección para devolver
                    siguiente_posicion = posicion_actual + direccion;
                elseif siguiente_posicion < 1
                    direccion = 1; % Invertir dirección para ir hacia adelante
                    siguiente_posicion = posicion_actual + direccion;
                end
                % Obtener la posición objetivo
                objetivoX = posiciones(siguiente_posicion, 1);
                objetivoY = posiciones(siguiente_posicion, 2);
                % Calcular la distancia a la posición objetivo
                distancia = sqrt((objetivoX - posX_dron_predefinido)^2 + (objetivoY - posY_dron_predefinido)^2);
                % Verificar si el dron ya ha llegado al objetivo
                if distancia < velocidad
                    posX_dron_predefinido = objetivoX;
                    posY_dron_predefinido = objetivoY;
                    posicion_actual = siguiente_posicion; % Actualizar la posición actual
                else
                    % Mover el dron en la dirección del objetivo
                    angulo = atan2(objetivoY - posY_dron_predefinido, objetivoX - posX_dron_predefinido);
                    posX_dron_predefinido = posX_dron_predefinido + velocidad * cos(angulo);
                    posY_dron_predefinido = posY_dron_predefinido + velocidad * sin(angulo);
                end
                traci.poi.setPosition(poiID_Predefinido,posX_dron_predefinido, posY_dron_predefinido)
                %% Movimiento del dron móvil basado en el mejor SNR promedio
                % Parámetros para el cálculo de movimiento
                num_aceleraciones = 5;  % Número de aceleraciones consideradas
                posible_dir = 36;  % Número de direcciones posibles (36 = 360° / 10°)
                resolucion_ang = 2 * pi / posible_dir;  % Resolución angular para las direcciones posibles
                angulo = 0:resolucion_ang:(2 * pi - resolucion_ang);  % Ángulos posibles
                posX2_movil = zeros(posible_dir, num_aceleraciones);
                posY2_movil = zeros(posible_dir, num_aceleraciones);
                promedio_SNRs = -inf(posible_dir, num_aceleraciones);
                aceleraciones = linspace(-acel_max, acel_max, num_aceleraciones);
                for k = 1:posible_dir
                    for a = 1:num_aceleraciones
                        % Calcula la nueva velocidad del dron considerando la aceleración
                        Vel_nueva = min(Vel_max, Vel_dron + aceleraciones(a) * tiempo);

                        % Calcula el desplazamiento en X y Y
                        disX = Vel_nueva * tiempo * cos(angulo(k)) + 0.5 * aceleraciones(a) * tiempo^2 * cos(angulo(k));
                        disY = Vel_nueva * tiempo * sin(angulo(k)) + 0.5 * aceleraciones(a) * tiempo^2 * sin(angulo(k));

                        % Calcula las nuevas posiciones del dron móvil
                        posX2_movil(k, a) = posX_dron_Movil + disX;
                        posY2_movil(k, a) = posY_dron_Movil + disY;

                        % Calcula el SNR promedio para las nuevas posiciones
                        total_SNR = 0;
                        count_vehicles = 0;
                        for j = 1:num_autos
                            if ~isnan(rxPositions(1, j))
                                txPosition = [posX2_movil(k, a); posY2_movil(k, a); simParameters.TxHeight];
                                rxPosition = rxPositions(:, j);
                                pathLoss = nrPathLoss(simParameters.PathLoss, simParameters.CarrierFrequency, simParameters.LOS, txPosition, rxPosition);
                                SNR = (simParameters.TxPower - 30) - pathLoss - 10 * log10(fftOccupancy) - 10 * log10(2 * N0^2);
                                total_SNR = total_SNR + SNR;
                                count_vehicles = count_vehicles + 1;
                            end
                        end
                        if count_vehicles > 0
                            promedio_SNRs(k, a) = total_SNR / count_vehicles;
                        end
                    end
                end
                if ~siguiendo_auto
                    % Elegir la dirección y aceleración que maximizan el SNR promedio
                    [~, idx] = max(promedio_SNRs(:));
                    [mejor_direccion, mejor_aceleracion_idx] = ind2sub(size(promedio_SNRs), idx);

                    Vel_dron = min(Vel_max, Vel_dron + aceleraciones(mejor_aceleracion_idx) * tiempo);
                    nueva_posX = posX2_movil(mejor_direccion, mejor_aceleracion_idx);
                    nueva_posY = posY2_movil(mejor_direccion, mejor_aceleracion_idx);
                else
                    % Si el dron está siguiendo al auto, moverlo a la posición del auto
                    nueva_posX = rxPositions(1, 1)+mov*1.5;
                    nueva_posY = rxPositions(2, 1)+mov;
                end

                % Mover el dron a la nueva posición
                traci.poi.setPosition(poiID_Movil, nueva_posX, nueva_posY);

                % Actualizar la posición del dron
                posX_dron_Tracking = nueva_posX;
                posY_dron_Tracking = nueva_posY;
                posX_veh = pos2D(1);
                posY_veh = pos2D(2);
                % % Actualizar la posición del vehículo en SUMO
                % % % posX_veh = posX_veh + mov*1.5; % Mover el vehículo 10 unidades a la derecha
                % % % posY_veh = posY_veh + mov;
                % % % traci.vehicle.moveToXY(vehicleID, '', 0, posX_veh, posY_veh, 58, 0);
                % Verificar si el vehículo está fuera de los límites y eliminarlo
                x_limite = 1430; % Define el límite en X
                if posX_veh >= x_limite
                    traci.vehicle.remove(vehicleID);
                    disp(['Vehículo ' vehicleID ' ha salido de la simulación.']);
                end

                % Si el dron está lo suficientemente cerca del auto, comenzar a seguirlo
                if ~siguiendo_auto
                    distancia_al_auto = sqrt((posX_dron_Tracking - rxPositions(1, 1))^2 + (posY_dron_Tracking - rxPositions(2, 1))^2);
                    if distancia_al_auto < 20 % Umbral para determinar si el dron está lo suficientemente cerca del auto
                        siguiendo_auto = true;
                    end
                end
            end
        end
    end


    traci.close();  % Cerrar la conexión con SUMO
    % Ajustar la longitud de las matrices de datos para cada vehículo
    maxSteps = max([length(promedioSNR_Predefinido), length(promedioSNR_Fijo), length(promedioSNR_Movil)]);

    % Rellenar las matrices de datos con NaN para igualar la longitud en todas
    % las simulaciones, en caso de que estas tengan distinta longitud

    for i = 1:num_autos
        SNRs_Predefinido(:, i) = padWithNaN(SNRs_Predefinido(:, i), [maxSteps, 1]);
        SNRs_Fijo(:, i) = padWithNaN(SNRs_Fijo(:, i), [maxSteps, 1]);
        SNRs_Movil(:, i) = padWithNaN(SNRs_Movil(:, i), [maxSteps, 1]);

        simThroughput_vehiculos_Predefinido{i} = padWithNaN(simThroughput_vehiculos_Predefinido{i}', [maxSteps, 1])';
        simThroughput_vehiculos_Fijo{i} = padWithNaN(simThroughput_vehiculos_Fijo{i}', [maxSteps, 1])';
        simThroughput_vehiculos_Movil{i} = padWithNaN(simThroughput_vehiculos_Movil{i}', [maxSteps, 1])';

        maxThroughput_vehiculos_Predefinido{i} = padWithNaN(maxThroughput_vehiculos_Predefinido{i}', [maxSteps, 1])';
        maxThroughput_vehiculos_Fijo{i} = padWithNaN(maxThroughput_vehiculos_Fijo{i}', [maxSteps, 1])';
        maxThroughput_vehiculos_Movil{i} = padWithNaN(maxThroughput_vehiculos_Movil{i}', [maxSteps, 1])';
    end

    % Ajustar la longitud de las matrices de datos para cada vehículo
    maxSteps = max([length(promedioSNR_Predefinido), length(promedioSNR_Fijo), length(promedioSNR_Movil)]);

    %Rellenar las matrices de datos con NaN para igualar la longitud en todas las simulaciones
    for i = 1:num_autos
        SNRs_Predefinido(:, i) = padWithNaN(SNRs_Predefinido(:, i), [maxSteps, 1]);
        SNRs_Fijo(:, i) = padWithNaN(SNRs_Fijo(:, i), [maxSteps, 1]);
        SNRs_Movil(:, i) = padWithNaN(SNRs_Movil(:, i), [maxSteps, 1]);

        simThroughput_vehiculos_Predefinido{i} = padWithNaN(simThroughput_vehiculos_Predefinido{i}', [maxSteps, 1])';
        simThroughput_vehiculos_Fijo{i} = padWithNaN(simThroughput_vehiculos_Fijo{i}', [maxSteps, 1])';
        simThroughput_vehiculos_Movil{i} = padWithNaN(simThroughput_vehiculos_Movil{i}', [maxSteps, 1])';

        maxThroughput_vehiculos_Predefinido{i} = padWithNaN(maxThroughput_vehiculos_Predefinido{i}', [maxSteps, 1])';
        maxThroughput_vehiculos_Fijo{i} = padWithNaN(maxThroughput_vehiculos_Fijo{i}', [maxSteps, 1])';
        maxThroughput_vehiculos_Movil{i} = padWithNaN(maxThroughput_vehiculos_Movil{i}', [maxSteps, 1])';
    end

    % Actualizar las longitudes de steps para las diferentes trayectorias
    steps = (0:maxSteps)/10;
    % Figures:
    % Trajectories
    fsize = 10;
    lsize = 2;
    %% Para gráficos de Bler y Throughput promedio

    simThroughput_vehiculos_Predefinido2 = cell2mat(simThroughput_vehiculos_Predefinido); % Convertir a matriz
    bler_predefinido2 = cell2mat(bler_predefinido); % Convertir a matriz

    % Calcular el promedio por fila ignorando NaN
    promedioThroughput_Predefinido = nanmean(simThroughput_vehiculos_Predefinido2, 2);
    promedioBler_Predefinido = nanmean(bler_predefinido2, 2);

    simThroughput_vehiculos_Fijo2 = cell2mat(simThroughput_vehiculos_Fijo); % Convertir a matriz
    bler_fijo2 = cell2mat(bler_fijo); % Convertir a matriz

    % Calcular el promedio por fila ignorando NaN
    promedioThroughput_Fijo = nanmean(simThroughput_vehiculos_Fijo2, 2);
    promedioBler_Fijo = nanmean(bler_fijo2, 2);

    simThroughput_vehiculos_Movil2 = cell2mat(simThroughput_vehiculos_Movil); % Convertir a matriz
    bler_movil2 = cell2mat(bler_movil); % Convertir a matriz

    % Calcular el promedio por fila ignorando NaN
    promedioThroughput_Movil = nanmean(simThroughput_vehiculos_Movil2, 2);
    promedioBler_Movil = nanmean(bler_movil2, 2);

    % Pasar a Mbps
    promedioThroughput_Movil = 1e-6*promedioThroughput_Movil/(simParameters.NFrames*10e-3);
    promedioThroughput_Predefinido = 1e-6*promedioThroughput_Predefinido/(simParameters.NFrames*10e-3);
    promedioThroughput_Fijo = 1e-6*promedioThroughput_Fijo/(simParameters.NFrames*10e-3);

    snr_all_iterations{iter,1} = promedioSNR_Movil;
    snr_all_iterations{iter,2} = promedioSNR_Predefinido;
    snr_all_iterations{iter,3} = promedioSNR_Fijo;

    bler_all_iterations{iter,1} = promedioBler_Movil;
    bler_all_iterations{iter,2} = promedioBler_Predefinido;
    bler_all_iterations{iter,3} = promedioBler_Fijo;

    throughput_all_iterations{iter,1} = promedioThroughput_Movil;
    throughput_all_iterations{iter,2} = promedioThroughput_Predefinido;
    throughput_all_iterations{iter,3} = promedioThroughput_Fijo;

end

% % Restar los valores del punto inicial
% X_relative = pos3D_history_Movil(:,1) - 1010.7;
% Y_relative = pos3D_history_Movil(:,2) - 767;
% 
% % Obtener trayectoria del vehículo 1 (puedes cambiar el índice si hay más)
% veh_tray = pos3D_history_vehiculos{1};
% X_veh = veh_tray(:,1) - 1010.7;
% Y_veh = veh_tray(:,2) - 767;
% 
% % Crear la figura
% figure;
% hold on;
% 
% % Graficar toda la trayectoria con línea de guiones más gruesa
% h1 = plot(X_relative, Y_relative, '--', 'Color', [0 0.4470 0.7410], 'LineWidth', 2);
% 
% % Marcar el punto inicial en verde sin coordenadas
% h2 = plot(X_relative(1), Y_relative(1), 'o', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'g', 'MarkerSize', 10);
% 
% % Marcar el punto final en rojo sin coordenadas
% h3 = plot(X_relative(end), Y_relative(end), 'o', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'r', 'MarkerSize', 10);
% 
% % Graficar trayectoria del vehículo en gris oscuro con línea sólida
% h4 = plot(X_veh, Y_veh, '-', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.5);
% 
% % Configuración del gráfico
% %title('Trajectory of UAV1: Mobile');
% xlabel('X [m]');
% ylabel('Y [m]');
% grid on;
% axis equal;
% 
% % Agregar la leyenda
% legend([h1 h2 h3 h4], {'Trajectory UAV1', 'Start', 'End', 'Vehicle trajectory'}, 'Location', 'best');
% 
% 
% hold off;

% % Restar los valores del punto inicial
% X_relative = pos3D_history_Movil(:,1) - 1010.7;
% Y_relative = pos3D_history_Movil(:,2) - 767;
% 
% % Obtener trayectoria del vehículo 1
% veh_tray = pos3D_history_vehiculos{1};
% X_veh = veh_tray(:,1) - 1010.7;
% Y_veh = veh_tray(:,2) - 767;
% 
% % Crear la figura
% figure;
% hold on;
% 
% % Graficar trayectoria del UAV móvil
% h1 = plot(X_relative, Y_relative, '--', 'Color', [0 0.4470 0.7410], 'LineWidth', 2);
% 
% % Graficar trayectoria del vehículo
% h4 = plot(X_veh, Y_veh, '-', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.5);
% 
% % Marcar punto inicial del UAV (azul)
% plot(X_relative(1), Y_relative(1), '^', 'MarkerFaceColor', [0 0.4470 0.7410], ...
%      'MarkerEdgeColor', 'k', 'MarkerSize', 8);
% 
% % Marcar punto final del UAV (rojo)
% plot(X_relative(end), Y_relative(end), 'v', 'MarkerFaceColor', [0 0.4470 0.7410], ...
%      'MarkerEdgeColor', 'k', 'MarkerSize', 8);
% 
% % Marcar punto inicial del vehículo (ocre)
% plot(X_veh(1), Y_veh(1), 's', 'MarkerFaceColor', [0.85 0.44 0.00], ...
%      'MarkerEdgeColor', 'k', 'MarkerSize', 8);
% 
% % Marcar punto inicial del vehículo (ocre)
% plot(X_veh(end), Y_veh(end), 's', 'MarkerFaceColor', [0.85 0.44 0.00], ...
%      'MarkerEdgeColor', 'k', 'MarkerSize', 8);
% 
% % Configuración del gráfico
% xlabel('X [m]');
% ylabel('Y [m]');
% grid on;
% axis equal;
% 
% % Agregar leyenda (sin los puntos individuales)
% legend([h1 h4], {'Trajectory UAV1', 'Vehicle trajectory'}, 'Location', 'best');
% 
% hold off;

% Restar los valores del punto inicial
X_relative = pos3D_history_Movil(:,1) - 1010.7;
Y_relative = pos3D_history_Movil(:,2) - 767;

% Obtener trayectoria del vehículo 1
veh_tray = pos3D_history_vehiculos{1};
X_veh = veh_tray(:,1) - 1010.7;
Y_veh = veh_tray(:,2) - 767;

% Crear la figura
figure;
hold on;
ocre = [0.85 0.33 0.10];
% Graficar trayectoria del UAV móvil
% Graficar trayectoria del vehículo
h4 = plot(X_veh, Y_veh, '-', 'Color', ocre, 'LineWidth', 1.5);
h1 = plot(X_relative, Y_relative, '--', 'Color', [0 0.4470 0.7410], 'LineWidth', 2);
% Marcar inicio y fin del UAV
plot(X_relative(1), Y_relative(1), '^', 'MarkerFaceColor', [0 0.4470 0.7410], 'MarkerEdgeColor', 'k', 'MarkerSize', 8);
plot(X_relative(end), Y_relative(end), 'v', 'MarkerFaceColor', [0 0.4470 0.7410], 'MarkerEdgeColor', 'k', 'MarkerSize', 8);
text(X_relative(1)+15, Y_relative(1), 'UAV Start', 'Color', [0 0.4470 0.7410], 'FontWeight', 'bold');
text(X_relative(end)-120, Y_relative(end)+15, 'UAV End', 'Color', [0 0.4470 0.7410], 'FontWeight', 'bold');

plot(X_veh(1), Y_veh(1), 's', 'MarkerFaceColor', ocre, 'MarkerEdgeColor', 'k', 'MarkerSize', 8);
plot(X_veh(end), Y_veh(end), 's', 'MarkerFaceColor', ocre, 'MarkerEdgeColor', 'k', 'MarkerSize', 8);
text(X_veh(1)+15, Y_veh(1), 'Vehicle Start', 'Color', ocre, 'FontWeight', 'bold');
text(X_veh(end)-150, Y_veh(end), 'Vehicle End', 'Color', ocre, 'FontWeight', 'bold');

% Configuración del gráfico
xlabel('X [m]');
ylabel('Y [m]');
grid on;
axis equal;

xlim([-280 420]);  % ← Ajusta estos valores según tus datos
ylim([-310 150]);

% Agregar leyenda solo para las trayectorias
legend([h1 h4], {'Trajectory UAV1', 'Vehicle trajectory'}, 'Location', 'best');
hold off;


%% Limpieza de datos
% --- Adaptar para SNRs ---
% Encontrar la longitud máxima de las filas en cada celda para SNRs
max_length_snr_movil = max(cellfun(@length, snr_all_iterations(:, 1)));
max_length_snr_predefinido = max(cellfun(@length, snr_all_iterations(:, 2)));
max_length_snr_fijo = max(cellfun(@length, snr_all_iterations(:, 3)));

% Rellenar con NaN y calcular el promedio ignorando NaN para SNRs
padded_snr_movil = cellfun(@(x) [x NaN(1, max_length_snr_movil - length(x))], snr_all_iterations(:, 1), 'UniformOutput', false);
promedio_snr_movil = mean(cell2mat(padded_snr_movil), 1, 'omitnan');

padded_snr_predefinido = cellfun(@(x) [x NaN(1, max_length_snr_predefinido - length(x))], snr_all_iterations(:, 2), 'UniformOutput', false);
promedio_snr_predefinido = mean(cell2mat(padded_snr_predefinido), 1, 'omitnan');

padded_snr_fijo = cellfun(@(x) [x NaN(1, max_length_snr_fijo - length(x))], snr_all_iterations(:, 3), 'UniformOutput', false);
promedio_snr_fijo = mean(cell2mat(padded_snr_fijo), 1, 'omitnan');

% --- Adaptar para BLER ---
% Encontrar la longitud máxima de las filas en cada celda para BLER (convertir a vector fila)
max_length_bler_movil = max(cellfun(@length, bler_all_iterations(:, 1)));
max_length_bler_predefinido = max(cellfun(@length, bler_all_iterations(:, 2)));
max_length_bler_fijo = max(cellfun(@length, bler_all_iterations(:, 3)));

% Convertir las celdas en vectores fila y rellenar con NaN
padded_bler_movil = cellfun(@(x) [x(:).' NaN(1, max_length_bler_movil - length(x))], bler_all_iterations(:, 1), 'UniformOutput', false);
promedio_bler_movil = mean(cell2mat(padded_bler_movil), 1, 'omitnan');

padded_bler_predefinido = cellfun(@(x) [x(:).' NaN(1, max_length_bler_predefinido - length(x))], bler_all_iterations(:, 2), 'UniformOutput', false);
promedio_bler_predefinido = mean(cell2mat(padded_bler_predefinido), 1, 'omitnan');

padded_bler_fijo = cellfun(@(x) [x(:).' NaN(1, max_length_bler_fijo - length(x))], bler_all_iterations(:, 3), 'UniformOutput', false);
promedio_bler_fijo = mean(cell2mat(padded_bler_fijo), 1, 'omitnan');

% --- Adaptar para Throughput ---
% Encontrar la longitud máxima de las filas en cada celda para Throughput (convertir a vector fila)
max_length_throughput_movil = max(cellfun(@length, throughput_all_iterations(:, 1)));
max_length_throughput_predefinido = max(cellfun(@length, throughput_all_iterations(:, 2)));
max_length_throughput_fijo = max(cellfun(@length, throughput_all_iterations(:, 3)));

% Convertir las celdas en vectores fila y rellenar con NaN
padded_throughput_movil = cellfun(@(x) [x(:).' NaN(1, max_length_throughput_movil - length(x))], throughput_all_iterations(:, 1), 'UniformOutput', false);
promedio_throughput_movil = mean(cell2mat(padded_throughput_movil), 1, 'omitnan');

padded_throughput_predefinido = cellfun(@(x) [x(:).' NaN(1, max_length_throughput_predefinido - length(x))], throughput_all_iterations(:, 2), 'UniformOutput', false);
promedio_throughput_predefinido = mean(cell2mat(padded_throughput_predefinido), 1, 'omitnan');

padded_throughput_fijo = cellfun(@(x) [x(:).' NaN(1, max_length_throughput_fijo - length(x))], throughput_all_iterations(:, 3), 'UniformOutput', false);
promedio_throughput_fijo = mean(cell2mat(padded_throughput_fijo), 1, 'omitnan');

steps = (0:maxSteps-1)/divisor;


%% SNR

figure;
hold on;
plot(steps, promedio_snr_movil, 'o-', 'MarkerSize',7,'LineWidth',1, 'Color',[0 0.4470 0.7410]);
plot(steps, promedio_snr_predefinido, 'o-', 'LineWidth',1,'Color',[0.8500 0.3250 0.0980]);
plot(steps, promedio_snr_fijo, 'o-','MarkerSize',7,'LineWidth', 1,'Color',[0.4660 0.6740 0.1880]);
current_ylim = ylim;
ylim([current_ylim(1), 16]); % Aumentar el límite superior en un 10%
xlim([0,45])
xlabel('Time (s)');
ylabel('Average SNR (dB)');

grid on;
legend('UAV1: mobile','UAV2: predefined','UAV3: static','Vehicle','Location','northeast');
set(gca, 'FontSize', 12); % Cambia el tamaño de los números en los ejes
hold off;
% figure;
% hold on;
% plot(promedio_snr_movil, promedio_bler_movil, '-o', 'MarkerSize',7,'LineWidth',1, 'Color',[0 0.4470 0.7410]);
% % plot(promedio_snr_predefinido,promedio_bler_predefinido, '-o', 'LineWidth',1,'Color',[0.8500 0.3250 0.0980]);
% % plot(promedio_snr_fijo,promedio_bler_fijo, '-o','MarkerSize',7,'LineWidth', 1,'Color',[0.4660 0.6740 0.1880]);
% current_ylim = ylim;
% % ylim([current_ylim(1), current_ylim(2) * 1.25]); % Aumentar el límite superior en un 10%
% % % xlim([0,46])
% xlabel('Average SNR (dB)');
% ylabel('Average Throughput (mbps)');
% 
% grid on;
% legend('UAV1: mobile','UAV2: predefined','UAV3: static','Vehicle','Location','northeast');
% hold off;



%% Bler

figure;
hold on;
plot(steps, promedio_bler_movil,'o-', 'MarkerSize',7,'LineWidth',1,'Color',[0 0.4470 0.7410]);
plot(steps, promedio_bler_predefinido,'o-', 'LineWidth',1,'Color',[0.8500 0.3250 0.0980]);
plot(steps, promedio_bler_fijo, 'o-','MarkerSize',7,'LineWidth', 1,'Color',[0.4660 0.6740 0.1880]);

ylim([0, 1]); % Aumentar el límite superior en un 10%
xlim([0,45])
xlabel('Time (s)');
ylabel('Average BLER');
grid on;

legend('UAV1: mobile','UAV2: predefined','UAV3: static','Vehicle','Location','northeast');
set(gca, 'FontSize', 12); % Cambia el tamaño de los números en los ejes
hold off;

%% Throughput

figure;
hold on;
plot(steps, promedio_throughput_movil,'o-', 'MarkerSize',7,'LineWidth',1,'Color',[0 0.4470 0.7410]);
plot(steps, promedio_throughput_predefinido,'o-', 'LineWidth',1,'Color',[0.8500 0.3250 0.0980]);
plot(steps, promedio_throughput_fijo, 'o-','MarkerSize',7,'LineWidth', 1,'Color',[0.4660 0.6740 0.1880]);
current_ylim = ylim;
ylim([0, 35]); % Aumentar el límite superior en un 10%
xlim([0,45])
xlabel('Time (s)');
ylabel('Average Throughput (Mbps)');
grid on;
legend('UAV1: mobile','UAV2: predefined','UAV3: static','Vehicle','Location','northeast');
set(gca, 'FontSize', 12); % Cambia el tamaño de los números en los ejes
hold off;

%% Local Functions
function validateNumLayers(simParameters)
numlayers = simParameters.PDSCH.NumLayers;
ntxants = simParameters.NTxAnts;
nrxants = simParameters.NRxAnts;
antennaDescription = sprintf('min(NTxAnts,NRxAnts) = min(%d,%d) = %d', ntxants, nrxants, min(ntxants, nrxants));
if numlayers > min(ntxants, nrxants)
    error('The number of layers (%d) must satisfy NumLayers <= %s', numlayers, antennaDescription);
end

if (numlayers > 2) && (numlayers == min(ntxants, nrxants))
    warning(['The maximum possible rank of the channel, given by %s, is equal to NumLayers (%d).' ...
        ' This may result in a decoding failure under some channel conditions.' ...
        ' Try decreasing the number of layers or increasing the channel rank' ...
        ' (use more transmit or receive antennas).'], antennaDescription, numlayers);
end
end


function estChannelGrid = getInitialChannelEstimate(carrier, nTxAnts, propchannel, dataType)
ofdmInfo = nrOFDMInfo(carrier);
chInfo = info(propchannel);
maxChDelay = chInfo.MaximumChannelDelay;

tmpWaveform = zeros((ofdmInfo.SampleRate / 1000 / carrier.SlotsPerSubframe) + maxChDelay, nTxAnts, dataType);

[~, pathGains, sampleTimes] = propchannel(tmpWaveform);

pathFilters = getPathFilters(propchannel);
offset = nrPerfectTimingEstimate(pathGains, pathFilters);

estChannelGrid = nrPerfectChannelEstimate(carrier, pathGains, pathFilters, offset, sampleTimes);
end


function estChannelGrid = precodeChannelEstimate(carrier, estChannelGrid, W)
[K, L, R, P] = size(estChannelGrid);
estChannelGrid = reshape(estChannelGrid, [K*L, R, P]);
estChannelGrid = nrPDSCHPrecode(carrier, estChannelGrid, reshape(1:numel(estChannelGrid), [K*L, R, P]), W);
estChannelGrid = reshape(estChannelGrid, K, L, R, []);
end


function plotLayerEVM(NSlots, nslot, pdsch, siz, pdschIndices, pdschSymbols, pdschEq)
persistent slotEVM;
persistent rbEVM;
persistent evmPerSlot;

if (nslot == 0)
    slotEVM = comm.EVM;
    rbEVM = comm.EVM;
    evmPerSlot = NaN(NSlots, pdsch.NumLayers);
    figure;
end
evmPerSlot(nslot + 1, :) = slotEVM(pdschSymbols, pdschEq);
subplot(2, 1, 1);
plot(0:(NSlots - 1), evmPerSlot, 'o-');
xlabel('Slot number');
ylabel('EVM (%)');
legend("layer " + (1:pdsch.NumLayers), 'Location', 'EastOutside');
title('EVM per layer per slot');

subplot(2, 1, 2);
[k, ~, p] = ind2sub(siz, pdschIndices);
rbsubs = floor((k - 1) / 12);
NRB = siz(1) / 12;
evmPerRB = NaN(NRB, pdsch.NumLayers);
for nu = 1:pdsch.NumLayers
    for rb = unique(rbsubs).'
        this = (rbsubs == rb & p == nu);
        evmPerRB(rb + 1, nu) = rbEVM(pdschSymbols(this), pdschEq(this));
    end
end
plot(0:(NRB - 1), evmPerRB, 'x-');
xlabel('Resource block');
ylabel('EVM (%)');
legend("layer " + (1:pdsch.NumLayers), 'Location', 'EastOutside');
title(['EVM per layer per resource block, slot #' num2str(nslot)]);

drawnow;
end


function paddedArray = padWithNaN(array, desiredSize)
% Esta función rellena una matriz con NaN para alcanzar el tamaño deseado
currentSize = size(array);
paddingSize = desiredSize - currentSize;
if paddingSize(1) > 0
    paddedArray = [array; NaN(paddingSize(1), currentSize(2))];
else
    paddedArray = array;
end
end

