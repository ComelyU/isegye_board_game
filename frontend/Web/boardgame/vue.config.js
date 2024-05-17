const { defineConfig } = require('@vue/cli-service')
module.exports = defineConfig({
  transpileDependencies: true,
  devServer: {
    allowedHosts: [
      'k10a706.p.ssafy.io',
      'localhost',
      '127.0.0.1'
    ]
  }
})
