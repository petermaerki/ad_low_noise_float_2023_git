# Registry for ad_low_noise_float_2023_decoder

## Links

* https://github.com/devpi/devpi

* https://pip.pypa.io/en/stable/cli/pip_install/#install-index-url
  --extra-index-url <url>
  
## PEP 503 – Simple Repository API

* https://peps.python.org/pep-0503/

* https://pydevtools.com/handbook/explanation/what-is-pep-503/

* https://pypi.org/simple

* https://github.com/simple-repository/simple-repository

* https://packaging.python.org/en/latest/guides/hosting-your-own-index/

* https://docs.astral.sh/uv/concepts/indexes/#-index-url-and-extra-index-url

* https://github.com/astral-sh/uv/issues/1819#issuecomment-2746736655
  Insecure Host

## Implementation

### simple-repository on www.maerki.com

/home/www/htdocs/maerki/hans/download/pymeas2019_noise/ad-low-noise-float-2023-decoder

```bash
scp *.whl www-data@www.maerki.com:/home/www/htdocs/maerki/hans/download/pymeas2019_noise/ad-low-noise-float-2023-decoder
ad_low_noise_float_2023_decoder-0.1.3-cp312-cp312-manylinux_2_17_x86_64.manylinux_2_28_x86_64.whl               100%  110KB   1.3MB/s   00:00    
ad_low_noise_float_2023_decoder-0.1.3-cp312-cp312-musllinux_1_2_x86_64.whl                                     100% 1095KB   7.3MB/s   00:00    
ad_low_noise_float_2023_decoder-0.1.3-cp312-cp312-win_amd64.whl                                                100%   71KB   1.5MB/s   00:00    
ad_low_noise_float_2023_decoder-0.1.3-cp313-cp313-manylinux_2_17_x86_64.manylinux_2_28_x86_64.whl               100%  110KB   2.3MB/s   00:00    
ad_low_noise_float_2023_decoder-0.1.3-cp313-cp313-musllinux_1_2_x86_64.whl                                     100% 1096KB   8.0MB/s   00:00    
ad_low_noise_float_2023_decoder-0.1.3-cp313-cp313-win_amd64.whl                                                100%   71KB   1.5MB/s   00:00    
ad_low_noise_float_2023_decoder-0.1.3-cp39-cp39-manylinux_2_17_x86_64.manylinux_2_28_x86_64.whl                 100%  108KB   2.3MB/s   00:00    
ad_low_noise_float_2023_decoder-0.1.3-cp39-cp39-musllinux_1_2_x86_64.whl                                       100% 1094KB   8.0MB/s   00:00    
ad_low_noise_float_2023_decoder-0.1.3-cp39-cp39-win_amd64.whl                                                  100%   71KB   1.5MB/s   00:00  
```

https://www.maerki.com/hans/download/pymeas2019_noise


pyproject.toml
```
dependencies = [
    "isort",
    "ad_low_noise_float_2023_decoder",
]

[[tool.uv.index]]
name = "ad_low_noise_float_2023_decoder"
url = "https://www.maerki.com/hans/download/pymeas2019_noise"

[tool.uv]
allow-insecure-host = ["www.maerki.com"]
```
