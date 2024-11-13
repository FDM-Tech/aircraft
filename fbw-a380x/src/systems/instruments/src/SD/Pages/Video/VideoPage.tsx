import React from 'react';
import { PageTitle } from '../Generic/PageTitle';
import MapViewWasmModule from './MapViewWasmModule.wasm';

import '../../../index.scss';

export const VideoPage = () => {
  //const MapViewAircraft = MapViewAircraft();

  <PageTitle x={6} y={29}>
    VIDEO
  </PageTitle>;

  {
    MapViewWasmModule.wasm_gauge('MapView2D');
  }
};
