# 临时脚本：模拟"没装 OpenCV"，验证 detect_from_image.py 只打印安装提示
import builtins
import runpy

_real = builtins.__import__


def fake(name, *a, **k):
    if name.split('.')[0] in ('cv2', 'numpy'):
        raise ImportError('No module named ' + repr(name))
    return _real(name, *a, **k)


builtins.__import__ = fake
try:
    runpy.run_path('detect_from_image.py', run_name='__main__')
except SystemExit as e:
    print('SystemExit code =', e.code)
