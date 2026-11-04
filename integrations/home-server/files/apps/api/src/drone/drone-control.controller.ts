import {
  BadRequestException, Controller, ForbiddenException, Get, Param, Post, Req, Res, UseGuards
} from "@nestjs/common";
import type { RawBodyRequest } from "@nestjs/common";
import type { Request, Response } from "express";
import { DroneControlGuard } from "./drone-control.guard";
import { DroneControlGateway, type ControlResource } from "./drone-control.gateway";

@UseGuards(DroneControlGuard)
@Controller("drone/control")
export class DroneControlController {
  constructor(private readonly gateway: DroneControlGateway) {}

  @Get(":resource")
  async read(@Param("resource") resource: string, @Res() response: Response) {
    if (!["status", "window", "frame"].includes(resource)) {
      throw new BadRequestException("invalid_controller_resource");
    }
    this.reply(response, await this.gateway.forward("GET", resource as ControlResource));
  }

  @Post(":resource")
  async write(@Param("resource") resource: string, @Req() request: RawBodyRequest<Request>,
              @Res() response: Response) {
    if (!["intent", "camera"].includes(resource)
        || request.headers["content-type"]?.split(";")[0] !== "application/json"
        || !request.rawBody) throw new BadRequestException("invalid_controller_request");
    if (request.headers.origin && request.headers.origin !== process.env.WEB_ORIGIN) {
      throw new ForbiddenException("invalid_controller_origin");
    }
    this.reply(response, await this.gateway.forward("POST", resource as ControlResource, request.rawBody));
  }

  private reply(response: Response, value: Awaited<ReturnType<DroneControlGateway["forward"]>>) {
    response.status(value.status);
    response.setHeader("Cache-Control", "no-store");
    response.setHeader("X-Content-Type-Options", "nosniff");
    for (const name of ["content-type", "x-neo2-connection-generation",
      "x-neo2-frame-age-seconds", "x-neo2-frame-id"]) {
      const valueHeader = value.headers[name];
      if (typeof valueHeader === "string") response.setHeader(name, valueHeader);
    }
    response.setHeader("Content-Length", value.body.length);
    response.end(value.body);
  }
}
