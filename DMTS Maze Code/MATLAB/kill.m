function kill
% kill all figure windows

delete(findall(0,'Type','figure'))
delete(instrfindall)
delete(timerfindall)

end