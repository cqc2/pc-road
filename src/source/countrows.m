function rows = countrows(pointCloudFilePath)
    if (isunix) % Linux系统提供了wc命令可以直接使用
        % 使用syetem函数可以执行操作系统的函数
        % 比如window中dir，linux中ls等
        [~, numstr] = system( ['wc -l ', pointCloudFilePath] );
        rows=str2double(numstr);
    elseif (ispc) % Windows系统可以使用perl命令
        if exist('countlines.pl','file')~=2
            % perl文件内容很简单就两行
            % while (<>) {};
            % print $.,"\n";
            fid=fopen('countlines.pl','w');
            fprintf(fid,'%s\n%s','while (<>) {};','print $.,"\n";');
            fclose(fid);
        end
        % 执行perl脚本
        rows=str2double( perl('countlines.pl',pointCloudFilePath) );
    end
end