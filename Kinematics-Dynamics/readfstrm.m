function [outSignal, outTimeInterval] = readfstrm(inFilename)
%READFSTRM Load a MOOG FCS Stream (FSTRM) file from disk
% [SIGNAL, TIMEINTERVAL]=READFSTRM(FILENAME)
%
% INPUT PARAMETERS
%
%   inFilename         Name of file to be loaded. 
%                      Notice an frf .fstrm can only be imported when
%                      its .info file is available. Otherwise the I/O 
%                      structure, i.e., number of inputs and outputs,
%                      is not known.
%
% OUTPUT PARAMETERS
%
%   outSignal          Dependent on the data in the file, outSignal will be
%                      a @sig object or a @frd object
%   outTimeInterval    sampling time of the data

[~, ~, ext] = fileparts(inFilename);
if isempty(ext)
    ext = '.fstrm';
    inFilename = [inFilename, ext];
elseif ~strcmp(ext,'.fstrm')
    error(['Unexpected file extension specified - ''', ext, ''''] )
end

fileInfo=[inFilename '.info'];
isComplex = 'f';
if exist(fileInfo) == 2
    fileInfoRoot = xmlread(fileInfo);
    fastArrayNode = fileInfoRoot.getElementsByTagName('FastArray').item(0);
    isComplex = fastArrayNode.getAttribute('Complex');
end

if strcmpi(isComplex,'t') % FRD model
    dim = sscanf(char(fastArrayNode.getAttribute('Dimension')),'%d %d %d');
    if length(dim) == 2, % 2D array
        % insert a third dimension
        dim = [dim(1) 1 dim(end)]; 
    end
    lineMaxNumber = dim(1) * dim(2)*2 + 7; % *2 - real and imag ; 6 - 6 lines Header
    
    FID=fopen(inFilename);
    i=0;
    OutputName = cell(dim(1)*dim(2),1);
    OutputUnit = OutputName;
    InputName  = OutputName;
    InputUnit  = OutputName;
    
    Nch = 1;
    while i < lineMaxNumber
        i=i+1;
        tline = fgets(FID);
        antw=strfind(tline,'TimeInterval');
        if ~isempty(antw)
            enddt=strfind(tline,char(10));
            outTimeInterval=str2double(tline(14:enddt-1));
            SampleFreq=round(10/outTimeInterval);
            outTimeInterval=10/SampleFreq;
        end

        if ~isempty(strfind(tline,'Channel'))
            EqualSignPos = strfind(tline,'=');
            if ~isempty(EqualSignPos)
                tline = tline(EqualSignPos(1)+1:end);
            else
                error('readfstrm: file header does not copmly with specifications ');
            end
            
            % locate vertiical bars used as sperators
            OutputNamesStr = strtok(tline,';');
            verticalBarInd = strfind(OutputNamesStr,'|');
            
            % OutputNames
            if ~isempty(verticalBarInd) % && verticalBarInd(1) <= unit
                tmpstr=OutputNamesStr(1:verticalBarInd(1)-1);
            else
                tmpstr=OutputNamesStr;
            end
            if isempty(tmpstr)
                OutputName{Nch,1} = '';
            else
                OutputName{Nch,1} = tmpstr;
            end
            
            % InputNames
            if ~isempty(verticalBarInd) % && verticalBarInd(1) <= unit
                if length(verticalBarInd) > 1
                    InputName{Nch,1}=OutputNamesStr(verticalBarInd(1)+1:verticalBarInd(2)-1);
                else
                    InputName{Nch,1}=OutputNamesStr(verticalBarInd(1)+1:end);
                end
            else
                InputName{Nch,1} = tmpstr;
            end
            
            Ind2 = strfind(tline,';Unit=');
            OutputUnitStr = tline(Ind2+6:end); % 6 = length('Unit= ')
            OutputUnitStr = strtok(OutputUnitStr,';'); % remove the tail ';Format=' 

            verticalBarInd = strfind(OutputUnitStr,'|');
            
            % OutputNames
            if ~isempty(verticalBarInd) % && verticalBarInd(1) <= unit
                tmpstr=OutputUnitStr(1:verticalBarInd(1)-1);
            else
                tmpstr=OutputUnitStr;
            end
            if isempty(tmpstr)
                OutputUnit{Nch,1} = '';
            else
                OutputUnit{Nch,1} = tmpstr;
            end
            
            % InputNames
            if ~isempty(verticalBarInd) % && verticalBarInd(1) <= unit
                if length(verticalBarInd) > 1
                    InputUnit{Nch,1}=OutputUnitStr(verticalBarInd(1)+1:verticalBarInd(2)-1);
                else
                    InputUnit{Nch,1}=OutputUnitStr(verticalBarInd(1)+1:end);
                end
            else
                InputUnit{Nch,1} = tmpstr;
            end
            
            Nch=Nch+1;
        end
        if length(tline)>=6
            if strcmp(tline(1:6),'[Data]')
                if (tline(end) == 13)
                    fseek(FID,-1,'cof');
                end
                break
            end
        end
    end
    Nch = Nch -1;
    
    % remove the dublicated entries - real and imaginary
    OutputName  = OutputName(1:2:end,1) ;
    OutputUnit	= OutputUnit(1:2:end,1) ;
    % remove the dublicated entries - in an frf model with multiple inputs
    OutputName  = OutputName(1:dim(2):end,1) ;
    OutputUnit  = OutputUnit(1:dim(2):end,1) ;
    
    bulkdata    = fread(FID,[Nch inf],'float32')';
    
    realdata    = bulkdata(:,1:2:size(bulkdata,2)-1);%*0+repmat(1:100,H.N,1);
    imagdata    = bulkdata(:,2:2:size(bulkdata,2));%*0+repmat(201:300,H.N,1);
    complexdata = complex(realdata, imagdata);
    
    rd = reshape(complexdata,[dim(3) dim(2) dim(1)]);
    tmp = zeros(dim(1), dim(2), dim(3));
    for dim3Indx = 1:dim(end)
        tmp(:,:,dim3Indx) = squeeze(rd(dim3Indx,:,:)).';
    end
    rd = tmp;
    
    freqs=linspace(0,1/outTimeInterval/2,dim(end))*2*pi;

    outSignal = frd(rd,freqs);
    set(outSignal,'Ts',outTimeInterval,...
               'OutputName',OutputName,'OutputUnit',OutputUnit);
    
    if ~isempty(InputName) % i.e., exclude drive_psd and response_psd
        % remove the dublicated entries - real and imaginary
        InputName = InputName(1:2:end,1) ;
        InputUnit = InputUnit(1:2:end,1) ;
        % remove the dublicated entries - in an frf model with multiple inputs
        InputName = InputName(1:dim(2),1) ;
        InputUnit = InputUnit(1:dim(2),1) ;
        % assign to frd object
        set(outSignal,'InputName',InputName,'InputUnit',InputUnit);
    end
    
    fclose(FID);
else % signal

    FID=fopen(inFilename);
    i=0;
    outSignal.OutputName=[];
    outSignal.OutputUnit=[];
    Nch=0;
    while i<1000
        i=i+1;
        tline = fgets(FID);
        antw=strfind(tline,'TimeInterval');
        if ~isempty(antw)
            enddt=strfind(tline,char(10));
            outTimeInterval=str2double(tline(14:enddt-1));
        end

        if ~isempty(strfind(tline,'Channel '))
            isind=strfind(tline,'=')+1;
            unit=strfind(tline,';Unit=')-1;
            outSignal.OutputName{size(outSignal.OutputName,1)+1,1}=tline(isind(1):unit);
            isind2=strfind(tline,';Unit=')+6;
            format=strfind(tline,';Format=Float;')-1;
            outSignal.OutputUnit{size(outSignal.OutputUnit,1)+1,1}=tline(isind2:format);
            Nch=Nch+1;
        end


        if length(tline)>=6
            if strcmp(tline(1:6),'[Data]')
                if length(tline) > 7
                    fseek(FID,7-length(tline),'cof');
                end
                break
            end
        end

    end

    data=fread(FID,[Nch inf],'float32')';

    outSignal.OutputData=data;
    outSignal.Ts=outTimeInterval;
    fclose(FID);
end
